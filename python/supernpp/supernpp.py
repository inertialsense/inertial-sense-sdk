#!/usr/bin/python3

import os
import re
from subprocess import Popen
from os.path import normpath, basename
from threading import Thread
import time
from pathlib import Path
import shutil
import sys
import threading
import yaml

sys.path.insert(1, '../../SDK/python/logInspector')
sys.path.insert(1, '../logInspector')
sys.path.insert(1, '..')

from logReader import Log

class SuperNPP():
    def __init__(self, params_filename=None, serials=['ALL'], startMode=0):		# start mode 0=hot, 1=cold, 2=factory
        self.config_serials = serials
        self.logs_file = None

        # Open and read the YAML file
        with open(params_filename, 'r') as file:
            self.params = yaml.safe_load(file)
            if "directory" not in self.params:
                directory = os.path.dirname(params_filename)
                self.params["directory"] = str(directory)
            if not os.path.isdir(self.params["directory"]):
                print("Directory does not exist: ", self.params["directory"])
                exit(1)
            # Add full path to list of logs
            self.params["logs"] = [str(Path(self.params["directory"]) / Path(s)) for s in self.params["logs"]]

        self.startMode = startMode
        options_upper = ",".join(self.params["run_test"]).upper()
        self.testIMX = "IMX" in options_upper
        self.testGPX = "GPX" in options_upper
        self.logs = []
        self.log = Log()
        self.passResults = []
        self.failResults = []
        self._key_lock = threading.Lock()

        self.results_filename = self.resultsFilename(self.params)
        try:
            os.remove(self.results_filename)      # Remove old file
        except OSError:
            pass

        print("=====================  Init SuperNPP  =====================")
        print("  Directory:      ", self.params['directory'])
        print("  Logs:           ", self.params["logs"])
        print("  config_serials: ", self.config_serials)
        print("  startMode:      ", self.startMode)
        self.remove_post_processed_dirs(self.params["logs"])
        self.findLogFiles(self.params["logs"])
            
    def resultsFilename(self, params=None):
        if params is None:
            params = self.params
        results_filename =  os.path.normpath(os.path.join(self.params["directory"], self.params["results_directory"] , self.params["name"] + "_results.txt"))
        return results_filename

    def getSerialNumbers(self):
        return self.log.getSerialNumbers()

    def protocolVersion(self):
        return self.log.protocolVersion()

    def exitHack(self):
        self.log.exitHack()

    def findLogFiles(self, directories):
        if isinstance(directories, str):
            directories = [directories]  # Handle single string input just in case

        for directory in directories:
            self._findLogFilesRecursive(directory, self.params.copy())

    def _findLogFilesRecursive(self, directory, params={}):
        try:
            items = os.listdir(directory)
        except FileNotFoundError:
            print(f"Directory not found: {directory}")
            exit(1)
        except PermissionError:
            print(f"Permission denied: {directory}")
            exit(1)

        yaml_file = os.path.join(directory, "params.yaml")
        if os.path.isfile(yaml_file):
            with open(yaml_file, 'r') as file:
                override_params = yaml.safe_load(file)
                params.update(override_params)

        for item in items:
            item_path = os.path.join(directory, item)
            if os.path.isfile(item_path):
                # Directory contains .raw or .dat file
                if (".dat" in item or ".raw" in item) and "base_station.raw" not in item:
                    if os.path.basename(directory) in self.params["blacklist_logs"]:
                        print("Excluding blacklisted log: " + os.path.basename(directory))
                    else:
                        print("Adding log: " + directory)
                        params["directory"] = directory
                        self.logs.append(params.copy())
                    break
            elif os.path.isdir(item_path):
                if "post_processed" in item:
                    continue
                self._findLogFilesRecursive(item_path, params.copy())

    def remove_post_processed_dirs(self, base_dirs):
        """
        Recursively remove all directories named 'post_processed' in the specified base directories.

        Args:
            base_dirs (list of str): The paths to the base directories to search within.
        """
        if isinstance(base_dirs, str):
            base_dirs = [base_dirs]  # Allow fallback for single string input

        for base_dir in base_dirs:
            for root, dirs, files in os.walk(base_dir, topdown=False):
                for dir_name in dirs:
                    if dir_name == "post_processed":
                        dir_path = os.path.join(root, dir_name)
                        try:
                            shutil.rmtree(dir_path)
                            print(f"Removed directory: {dir_path}")
                        except Exception as e:
                            print(f"Failed to remove {dir_path}: {e}")

    def run_reprocess(self):
        if not self.params["reprocess"]:
            print("Reprocess disabled")
            return

        print('  log count: ' + str(len(self.logs)))
        for log in self.logs:
            print("   " + log["directory"])
        time.sleep(2)	# seconds

        # Start threads
        threads = [Thread(target=self.reprocess_log, args=(log["directory"], self.config_serials, yaml.dump(log).replace('\n', ';'))) for log in self.logs]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def run_tests(self):
        if self.testIMX: self.run_report(self.log.runImxPerformanceReport)
        if self.testGPX: self.run_report(self.log.runGpxPerformanceReport)

    def run_report(self, runReportFunc):
        results = []
        directories = [log["directory"] for log in self.logs]
        parent_dir = os.path.commonpath(directories)
        results.append(parent_dir + "\n")

        for log in self.logs:
            log["directory"] = os.path.normpath(str(log["directory"]))
            if self.params["reprocess"]:
                log["directory"] = os.path.normpath(str(log["directory"]) + "/post_processed")
            rel_dir = os.path.relpath(log["directory"], parent_dir)
            nppPrint("\nRun Report: " + log["directory"])

            ### Compute Performance report ##################################################
            if self.log.load(log["directory"]):
                # Compute and output performance report
                failures = runReportFunc(log)
                if failures:
                    results.append("[FAILED] " + rel_dir)
                    if isinstance(failures, list):
                        results.append("\n".join("         " + line for line in failures))
                    self.failResults.append(rel_dir)
                else:
                    results.append("[PASSED] " + rel_dir)
                    self.passResults.append(rel_dir)
            else:
                results.append("[NODATA] " + rel_dir)
            ### Compute Performance report ##################################################
        with open(self.results_filename, "w") as f:
            f.write("\n".join(results))

        output = '-------------------------------------------------------------\n'
        output += os.path.basename(self.results_filename) + "\n"
        with open(self.results_filename, 'r') as file:
            output += file.read()    
            output += "\n"
        output += '-------------------------------------------------------------\n'
        print(output)

    def reprocess_log(self, folder, config_serials, params_str):
        # Find the serial numbers in the log
        (folder, subdir) = os.path.split(folder)

        # Find serial numbers, and determine the log type
        logType = "DAT"
        if config_serials == ["ALL"]:
            serials = []
            for file in os.listdir(os.path.join(folder,subdir)):
                if (".dat" in file or ".raw" in file) and (not "base_station.raw" in file):
                    if ".dat" in file:
                        logType = "DAT"
                    elif ".raw" in file:
                        logType = "RAW"

                    serNum = int(re.sub('[^0-9]','', file.split("_")[1]))
                    if serNum and (serNum not in serials):
                        serials.append(serNum)
        else:
            serials = config_serials

        file_path = os.path.dirname(os.path.realpath(__file__))
        npp_build_folder = os.path.normpath(file_path + '../../../../cpp/NavPostProcess/build')
        if os.name == 'posix':  # Linux
            exename = './navpp'
        else:                   # Windows
            exename = 'navpp.exe'
            npp_build_folder += '/Release'
        cmds = [exename + ' -d "' + folder + '" -s ' + str(s) + " -sd " + subdir + " -l " + logType for s in serials]

        mode_suffix = {1: ' -mode COLD', 2: ' -mode FACTORY'}.get(self.startMode, '')
        for i in range(len(cmds)):
            cmds[i] += mode_suffix		            # Cold/Factory init
            cmds[i] += ' -kml'                      # Cold init, enable KML output
            cmds[i] += ' --outputoff'				# disable INS display output
            cmds[i] += ' --disableBaroFusion'		# disable barometer fusion
            cmds[i] += f' --params "{params_str}"'
        
        output = "Running NPP: \n"
        output += f"  Directory: {folder}\n"
        output += f"  Serials:   {', '.join(str(s) for s in serials)}\n"
        output += f"  Command(s): \n"
        for cmd in cmds:
            output += cmd + "\n"
        print(output)

        processes = [Popen(cmd, shell=True, cwd=npp_build_folder) for cmd in cmds]
        for p in processes:
            p.wait()

        print("All processes done!")

def buildNPP(npp_build_folder):
    if not os.path.exists(npp_build_folder):
        os.makedirs(npp_build_folder)

    print("building NPP")
    cmd = ['cmake .. -DCMAKE_BUILD_TYPE=Debug && make -j12 -l12']
    process = Popen(cmd, shell=True, cwd=npp_build_folder)
    process.wait()
 
def nppPrint(str):
    print(str)	# Comment out to disable output
    pass

def file_contains_string_count(file_path, search_string):
    count = 0
    with open(file_path, 'r') as file:
        content = file.read()
        count = content.count(search_string)
    return count

def lines_with_string(file_path, search_string):
    output = ""
    with open(file_path, 'r') as file:
        for line in file:
            if search_string in line:
                output += line
    return output

def string_case(filename, title_string, search_string):
    output = ""
    count = file_contains_string_count(filename, search_string)
    if count:
        output += title_string + " " + str(count)
        output += lines_with_string(filename, search_string)
    return output

if __name__ == "__main__":

    print("Running SuperNPP")
    npp_build_folder = "../../../cpp/NavPostProcess/build"
    # buildNPP(npp_build_folder)

    # 2nd argument: Log directory list file
    if len(sys.argv) < 2:
        exit(1)
    params_filename = sys.argv[1]
    if not os.path.isfile(params_filename):
        print("First parameter must the params yaml!")
        exit(1)

    # 3rd argument: Serial #s
    serials = ['ALL']
    if len(sys.argv) >= 4:
        serials = sys.argv[3]

    # Run Super NPP
    snpp = SuperNPP(params_filename, serials)
    snpp.run_reprocess()
    snpp.run_tests()

    # Open and read the YAML file
    with open(params_filename, 'r') as file:
        params = yaml.safe_load(file)
        directory = os.path.dirname(params_filename)
        testSummaryFilename = snpp.resultsFilename()
        output =  "====================  Super NPP Results  ====================\n"
        output += string_case(testSummaryFilename, "  Tests PASSED:", "[PASSED]")
        output += string_case(testSummaryFilename, "  Tests FAILED:", "[FAILED]")
        output += string_case(testSummaryFilename, "  Failed to Reprocess:", "[NODATA]")
        output += "\n"
        output += "============================================================="
        print(output)

    snpp.exitHack()

