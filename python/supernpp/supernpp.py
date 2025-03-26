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

sys.path.insert(1, '../../SDK/python/logInspector')
sys.path.insert(1, '../logInspector')
sys.path.insert(1, '..')

from logReader import Log

class SuperNPP():
    def __init__(self, test_name=None, logs_file=None, directory=None, serials=['ALL'], startMode=0, options=""):		# start mode 0=hot, 1=cold, 2=factory
        self.config_serials = serials
        self.test_name = test_name or "test_imx"
        self.logs_file = None
        if logs_file:
            self.logs_file = os.path.normpath(Path(logs_file))
            self.test_name = os.path.splitext(os.path.basename(logs_file))[0]
            self.directory = os.path.dirname(logs_file)
            with open(logs_file, 'r') as f:
                self.directories = [
                    os.path.normpath(os.path.join(self.directory, line.strip()))
                    for line in f
                    if line.strip() and not os.path.isabs(line.strip())
                ]
        elif directory:
            self.directory = os.path.normpath(Path(directory))
            self.directories = [ self.directory ]
        else:
            print("No log list file or directory provided.")
            return
        self.startMode = startMode
        options_upper = options.upper()
        self.computeIMX = "IMX" in options_upper
        self.computeGPX = "GPX" in options_upper
        self.subdirs = []
        self.log = Log()
        self.passResults = []
        self.failResults = []
        self._key_lock = threading.Lock()

        print("=====================  Init SuperNPP  =====================")
        if self.logs_file:
            print("  Log List: ", self.logs_file)
        print("  Directories: ", self.directories)
        print("  config_serials:", self.config_serials)
        print("  startMode: ", self.startMode)
        self.remove_post_processed_dirs(self.directories)
        self.findLogFiles(self.directories)
            
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
            self._findLogFilesRecursive(directory)

    def _findLogFilesRecursive(self, directory):
        try:
            entries = os.listdir(directory)
        except FileNotFoundError:
            print(f"Directory not found: {directory}")
            exit(1)
        except PermissionError:
            print(f"Permission denied: {directory}")
            exit(1)

        for file in entries:
            if (".dat" in file or ".raw" in file) and "base_station.raw" not in file:
                self.subdirs.append(directory)
                break

        for subdir in entries:
            subdir_path = os.path.join(directory, subdir)
            if not os.path.isdir(subdir_path):
                continue
            if "post_processed" in subdir:
                continue
            self._findLogFilesRecursive(subdir_path)

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


    def print_file_contents(self, file_path):
        with open(file_path, 'r') as file:
            for line in file:
                print(line, end='')  # end='' prevents adding extra newlines

    def run_process(self):
        print('  log count: ' + str(len(self.subdirs)))
        for subdir in self.subdirs:
            print("   " + subdir)
        time.sleep(2)	# seconds
        self.failResults = []
        self.passResults = []

        # Start threads
        threads = [Thread(target=self.reprocess_folder, args=(folder, self.config_serials)) for folder in self.subdirs]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        # Record list of logs to be processed
        self.results_filename = self.directory + "/results_" + self.test_name.split('_', 1)[1] + ".txt"
        try:
            os.remove(self.results_filename)      # Remove old file
        except OSError:
            pass

    def run_tests(self):
        if self.computeIMX: self.run_report(self.log.runImxPerformanceReport)
        if self.computeGPX: self.run_report(self.log.runGpxPerformanceReport)

    def run_report(self, runReportFunc):
        results = []
        parent_dir = os.path.commonpath(self.subdirs)
        results.append(parent_dir + "\n")

        for subdir in self.subdirs:
            sdir = os.path.normpath(str(subdir) + "/post_processed")
            rel_dir = os.path.relpath(sdir, parent_dir)
            nppPrint("   " + sdir)

            ### Compute Performance report ##################################################
            passRMS = 0
            if self.log.load(sdir):
                # Compute and output performance report
                passRMS = runReportFunc()
                if passRMS == 1:
                    results.append("[PASSED] " + rel_dir)
                    self.passResults.append(rel_dir)
                else:
                    results.append("[FAILED] " + rel_dir)
                    self.failResults.append(rel_dir)
            else:
                results.append("[NODATA] " + rel_dir)
            ### Compute Performance report ##################################################
        with open(self.results_filename, "w") as f:
            f.write("\n".join(results))

        print('-------------------------------------------------------------')
        print(os.path.basename(self.results_filename))
        self.print_file_contents(self.results_filename)
        print('-------------------------------------------------------------')

    def reprocess_folder(self, folder, config_serials):
        # Find the serial numbers in the log
        # subdir = os.path.basename(os.path.normpath(folder))
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

        print("serial numbers")
        print(serials)

        if os.name == 'posix':
            cmds = ['./navpp -d "' + folder + '" -s ' + str(s) + " -sd " + subdir + " -l " + logType for s in serials]
        file_path = os.path.dirname(os.path.realpath(__file__))
        npp_build_folder = os.path.normpath(file_path + '../../../../cpp/NavPostProcess/build')
        if os.name == 'posix':  # Linux
            exename = './navpp'
        else:                   # Windows
            exename = 'navpp.exe'
            npp_build_folder += '/Release'
        cmds = [exename + ' -d "' + folder + '" -s ' + str(s) + " -sd " + subdir + " -l " + logType for s in serials]

        if self.startMode == 1:
            for i in range(len(cmds)):
                cmds[i] += ' -mode COLD -kml'		# Cold init, enable KML output

        if self.startMode == 2:
            for i in range(len(cmds)):
                cmds[i] += ' -mode FACTORY -kml'	# Factory init, enable KML output

        for i in range(len(cmds)):
            cmds[i] += ' --outputoff'				# disable INS display output

        for i in range(len(cmds)):
            cmds[i] += ' --disableBaroFusion'		# disable barometer fusion

        print("Running NPP...")

        for cmd in cmds:
            print(cmd)

        processes = [Popen(cmd, shell=True, cwd=npp_build_folder) for cmd in cmds]
        for p in processes:
            p.wait()

        ### Compute RMS ##################################################
        # print("NPP done.  Running RMS calc...")
        # sdir = os.path.join(folder, subdir, "post_processed")
        #
        # self._key_lock.acquire()
        # log = Log()
        # log.load(sdir)
        # self._key_lock.release()
        #
        # # Compute and output RMS Report
        # log.calcImxInsStats()
        #
        # self._key_lock.acquire()
        # passRMS = log.printImxInsReport()
        # if passRMS == 1:
        # 	self.rmsPassResults.append(sdir)
        # 	print("RMS Test PASSED: " + sdir)
        # else:
        # 	self.rmsFailResults.append(sdir)
        # 	print("RMS Test FAILED: " + sdir)
        # self._key_lock.release()
        ### Compute RMS ##################################################

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

def print_lines_with_string(file_path, search_string):
    with open(file_path, 'r') as file:
        for line in file:
            if search_string in line:
                print(line, end='')  # end='' avoids adding extra newlines

def print_case(filename, title_string, search_string):
    count = file_contains_string_count(filename, search_string)
    if count:
        nppPrint(title_string + " " + str(count))
        print_lines_with_string(filename, search_string)

if __name__ == "__main__":

    print("Running SuperNPP")
    npp_build_folder = "../../../cpp/NavPostProcess/build"
    # buildNPP(npp_build_folder)

    # import yaml
    # file = open(os.path.expanduser("~/Documents/Inertial_Sense/config.yaml"), 'r')
    # config = yaml.load(file, Loader=yaml.FullLoader)
    # directory = config["directory"]
    # serials = config["serials"]

    # 2nd argument: Log directory list file
    if len(sys.argv) < 2:
        exit(1)
    logs_file = sys.argv[1]
    test_name = os.path.splitext(os.path.basename(logs_file))[0]
    directory = os.path.dirname(logs_file)

    # Debug
    # directory = '/home/walt/src/IS-src/scripts/../../goldenlogs/AHRS'
    # directory = os.path.join('C:/','_IS','goldenlogs','AHRS')
    # directory = os.path.join('C:/','_IS','goldenlogs')
    # directory = 'C:/_IS/goldenlogs'

    # serials = ""
    # directory = 'D:/Dropbox (Inertial Sense)/ISD/logs/202110/20211022_14_NAV_Drive_uins4_branch/20211022_145320'

    if not os.path.isfile(logs_file):
        print("First parameter must the path to the log list file!")
        exit(1)

    # 3rd argument: Compute Comparison Report
    if len(sys.argv) >= 3:
        options = sys.argv[2]
    else:
        options = "IMX" # Default to IMX

    # 3rd argument: Serial #s
    serials = ['ALL']
    if len(sys.argv) >= 4:
        serials = sys.argv[3]

    # Run Super NPP
    snpp = SuperNPP(
        logs_file=logs_file, 
        options=options,
        serials=serials)
    snpp.run_process()
    snpp.run_tests()

    testSummaryFilename = directory + "/results_" + test_name.split('_', 1)[1] + ".txt"
    nppPrint("\n")
    nppPrint("====================  Super NPP Results  ====================")
    print_case(testSummaryFilename, "  RMS Test PASSED:", "[PASSED]")
    print_case(testSummaryFilename, "  RMS Test FAILED:", "[FAILED]")
    print_case(testSummaryFilename, "  Failed to Reprocess:", "[NODATA]")
    nppPrint("=============================================================")

    snpp.exitHack()

