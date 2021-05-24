#!/usr/bin/env python3
import os
import argparse
import json
import yaml
import time
import subprocess


class Scenario:
    def __init__(self, width=None, height=None, start_row=None, start_col=None, goal_row=None, goal_col=None, dist=None):
        self.width = width
        self.height = height
        self.start_row = start_row
        self.start_col = start_col
        self.goal_row = goal_row
        self.goal_col = goal_col
        self.dist = dist


def read_tasks(path):
    tasks = []
    
    with open(path, 'r') as scenfile:
        assert(scenfile.readline().startswith('version'))
        for line in scenfile.readlines():
            tokens = line.strip().split()
            try:
                w, h = int(tokens[2]), int(tokens[3])
                start_col, start_row = int(tokens[4]), int(tokens[5])
                goal_col, goal_row = int(tokens[6]), int(tokens[7])
                dist = float(tokens[8])
                tasks.append(Scenario(
                    w, h,
                    start_row, start_col,
                    goal_row, goal_col,
                    dist
                ))
            except:
                print('Failed to parse scenario line: ' + line.strip())

    return tasks 


def status_report(msg, preserve=False):
    tcall = time.time()
    if preserve:
        print('\r\u001b[2K' + msg)
    elif tcall - status_report.last_call > 0.1:
        print('\r\u001b[2K' + msg, end='')
        status_report.last_call = tcall
status_report.last_call = 0.0


def run_benchmark(profile_path, resultsdir):
    try:
        with open(profile_path, 'r') as profile_file:
            profile = yaml.load(profile_file.read(), yaml.SafeLoader)
    except Exception as e:
        print('Failed to read benchmark profile file:', str(e))
        raise e

    for cfgname, config in profile.items():
        if 'skip' in config.keys() and config['skip'] == True:
            continue
        status_report(f'Running {resultsdir}')
        run_config(config, os.path.join(resultsdir, cfgname))


def run_config(config, resultsdir):
    try:
        map_path = config['map']
        scen_path = config['scen']
    except Exception as e:
        status_report(f'Expected "map" and "scen" fields in config: {config}', True)
        raise e
    timeout = config.get('timeout', 60.0)
    tasks = read_tasks(scen_path)
    for algo, func in [('astar', run_astar), ('rstar', run_rstar), ('mrastar', run_mrastar)]:
        if algo not in config:
            continue
        cfgs = config.get(algo, dict())
        if 'skip' in cfgs.keys():
            if cfgs['skip'] == True:
                continue
            del cfgs['skip']
        for pname, params in cfgs.items():
            if 'skip' in params.keys() and params['skip'] == True:
                continue
            for i, task in enumerate(tasks):
                subdir = os.path.join(resultsdir, algo, pname, str(i))
                status_report(f'Running {subdir}')
                os.makedirs(subdir, exist_ok=True)
                func(map_path, params, task, timeout, subdir)


def run_dump(params, timeout, logfile):
    try:
        result = subprocess.run(params, timeout=timeout)
        if result.returncode != 0:
            raise AssertionError(f'dump exited with non-zero status: {result.returncode}')
    except subprocess.TimeoutExpired as e:
        status_report(f'timeout: {logfile}', True)
        with open(logfile, 'w') as log:
            log.writelines(['timeout'])


def run_astar(map_path, params, task, timeout, resultsdir):
    logfile = os.path.join(resultsdir, 'log')
    weight = params.get('weight', 1.0)
    run_dump([
        './build/dump/dump',
        '--out', logfile,
        '--map', map_path,
        '--start', f'{task.start_row} {task.start_col}',
        '--goal', f'{task.goal_row} {task.goal_col}',
        '--astar',
        '--astar_weight', str(weight)
    ], timeout, logfile)


def run_rstar(map_path, params, task, timeout, resultsdir):
    logfile = os.path.join(resultsdir, 'log')
    try:
        delta = params['delta']
        k = params['k']
        weight = params['weight']
    except:
        msg = f'Expected "delta", "k", "weight" fields in R* parametrization: {params}'
        status_report(msg, True)
        raise AssertionError(msg)
    ti_factor = params.get('threshold_inflation_factor', 5.0)
    smart_iters_coef = params.get('smart_iters_coef', 8)
    range_low_coef = params.get('range_low_coef', 0.0)
    run_dump([
        './build/dump/dump',
        '--out', logfile,
        '--map', map_path,
        '--start', f'{task.start_row} {task.start_col}',
        '--goal', f'{task.goal_row} {task.goal_col}',
        '--rstar',
        '--rstar_delta', str(delta),
        '--rstar_k', str(k),
        '--rstar_weight', str(weight),
        '--rstar_ti_factor', str(ti_factor),
        '--rstar_smart_iters_coef', str(smart_iters_coef),
        '--rstar_range_low_coef', str(range_low_coef)
    ], timeout, logfile)


def run_mrastar(map_path, params, task, timeout, resultsdir):
    logfile = os.path.join(resultsdir, 'log')
    try:
        cell_sizes = params['cell_sizes']        
        weight = params['weight']
        suboptimality_coef = params['suboptimality_coef']
    except:
        msg = f'Expected "cell_sizes", "weight", "suboptimality_coef" fields in MRA* parametrization: {params}'
        status_report(msg, True)
        raise AssertionError(msg)
    choose_queue_method = params.get('choose_queue_method', 'round_robin')
    thompson_history_coef = params.get('thompson_history_coef', 10)
    run_dump([
        './build/dump/dump',
        '--out', logfile,
        '--map', map_path,
        '--start', f'{task.start_row} {task.start_col}',
        '--goal', f'{task.goal_row} {task.goal_col}',
        '--mrastar',
        '--mrastar_cell_sizes', ','.join(map(str, cell_sizes)),
        '--mrastar_weight', f'{weight}',
        '--mrastar_suboptimality_coef', f'{suboptimality_coef}',
        '--mrastar_choose_queue_method', f'{choose_queue_method}',
        '--mrastar_thompson_history_coef', f'{thompson_history_coef}'
    ], timeout, logfile)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Benchmark tool')
    parser.add_argument('profile', type=str, help='path to the benchmarking profile, see file examples')
    parser.add_argument('--out', type=str, default='bench-results/', required=False, help='path to put results to')
    args = parser.parse_args()

    run_benchmark(args.profile, args.out)
    status_report('Done.', True)
