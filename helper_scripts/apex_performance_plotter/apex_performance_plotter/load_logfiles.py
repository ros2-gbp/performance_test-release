# Copyright 2020 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import itertools
import json

import pandas


def load_logfile(filename):
    """Load logfile into header dictionary and pandas dataframe."""
    if filename.lower().endswith(".csv"):
        return load_logfile_csv(filename)
    elif filename.lower().endswith(".json"):
        return load_logfile_json(filename)


def load_logfile_csv(filename):
    with open(filename) as source:
        header = {}
        for item in itertools.takewhile(lambda x: not x.startswith('---'), source):
            if not item.strip():  # Don't care about whitespace-only lines
                continue
            try:
                key = item.split(':')[0].strip()
                value = item.split(':', maxsplit=1)[1].strip()
                header[key] = value
            except Exception:
                print('Error trying to parse header line "{}"'.format(item))
                raise
        dataframe = pandas.read_csv(source, sep='[ \t]*,[ \t]*', engine='python')
        unnamed = [col for col in dataframe.keys() if col.startswith('Unnamed: ')]
        if unnamed:
            dataframe.drop(unnamed, axis=1, inplace=True)
    print(header)
    print(dataframe)
    return header, dataframe


def load_logfile_json(filename):
    with open(filename) as source:
        header = json.load(source)
        qos_template = "Reliability: {rel} Durability: {dur} History kind: {hist} History depth: {depth}"
        qos_rendered = qos_template.format(
            rel=header['qos_reliability'],
            dur=header['qos_durability'],
            hist=header['qos_history_kind'],
            depth=header['qos_history_depth'])
        header['QOS'] = qos_rendered
        header['Logfile name'] = filename
        header['Experiment id'] = header['id']
        header['Communication mean'] = header['com_mean_str']
        header['Performance Test Version'] = header['perf_test_version']
        header['Publishing rate'] = header['rate']
        header['Topic name'] = header['topic_name']
        header['Number of publishers'] = header['number_of_publishers']
        header['Number of subscribers'] = header['number_of_subscribers']
        header['Maximum runtime (sec)'] = header['max_runtime']
        header['DDS domain id'] = header['dds_domain_id']

        dataframe = pandas.json_normalize(header, 'analysis_results')
        dataframe['T_experiment'] = dataframe['experiment_start'] / 1000000000
        dataframe['latency_min (ms)'] = dataframe['latency_min'] * 1000
        dataframe['latency_max (ms)'] = dataframe['latency_max'] * 1000
        dataframe['latency_mean (ms)'] = dataframe['latency_mean'] * 1000
        dataframe['latency_variance (ms)'] = dataframe['latency_variance'] * 1000
        dataframe['ru_maxrss'] = dataframe['sys_tracker_ru_maxrss']
        dataframe['ru_minflt'] = dataframe['sys_tracker_ru_minflt']
        dataframe['ru_majflt'] = dataframe['sys_tracker_ru_majflt']
        dataframe['ru_nivcsw'] = dataframe['sys_tracker_ru_nivcsw']
        dataframe['cpu_usage (%)'] = dataframe['cpu_info_cpu_usage']

        del header['analysis_results']
        print(header)
        print(dataframe)
        return header, dataframe


def load_logfiles(logfiles):
    """Load logfiles into header dictionaries and pandas dataframes."""
    headers = []
    dataframes = []

    for logfile in logfiles.value:
        header, dataframe = load_logfile(logfile)
        headers.append(header)
        dataframes.append(dataframe)

    return headers, dataframes
