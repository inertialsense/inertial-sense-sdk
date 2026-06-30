#!/usr/bin/env python

import configparser
config = configparser.ConfigParser()
config.read('setup.cfg')

old_version = config['metadata']['version']
(major, minor, patch) = old_version.split('.')

patch = str(int(patch) + 1)
new_version = major + '.' + minor + '.' + patch
config['metadata']['version'] = new_version

with open('setup.cfg', 'w') as configfile:
    config.write(configfile)