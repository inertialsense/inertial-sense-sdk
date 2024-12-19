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
    def __init__(self, directory, config_serials, startMode=0, computeRMS=0):		# start mode 0=hot, 1=cold, 2=factory
        self.config_serials = config_serials
        self.directory = os.path.normpath(Path(directory))
        self.startMode = startMode
        self.computeRMS = computeRMS
        self.subdirs = []
        self.log = Log()
        self.rmsPassResults = []
        self.rmsFailResults = []
        self._key_lock = threading.Lock()

        print("=====================  Init SuperNPP  =====================")
        print("  Directory: ", self.directory)
        print("  config_serials:", self.config_serials)
        print("  startMode: ", self.startMode)
        self.remove_post_processed_dirs(self.directory)
        self.findLogFiles(self.directory)
            
    def getSerialNumbers(self):
        return self.log.getSerialNumbers()

    def protocolVersion(self):
        return self.log.protocolVersion()

    def exitHack(self):
        self.log.exitHack()

    def findLogFiles(self, directory):
        # print("findLogFiles: ", directory)
        for file in os.listdir(directory):
            if (".dat" in file or ".raw" in file) and (not "base_station.raw" in file):
                self.subdirs.append(directory)
                break
        # Recursively search for data in sub directories
        for subdir in os.listdir(directory):
            subdir2 = os.path.join(directory, subdir)
            if not os.path.isdir(subdir2):
                continue
            if "post_processed" in subdir:
                continue
            self.findLogFiles(subdir2)

    def remove_post_processed_dirs(self, base_dir):
        """
        Recursively remove all directories named 'post_processed' in the specified base directory.

        Args:
            base_dir (str): The path to the base directory to search within.
        """
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

    def run(self):
        print('  log count: ' + str(len(self.subdirs)))
        for subdir in self.subdirs:
            print("   " + subdir)
        time.sleep(2)	# seconds
        self.rmsFailResults = []
        self.rmsPassResults = []

        # Start threads
        threads = [Thread(target=self.run_log_folder, args=(folder, self.config_serials)) for folder in self.subdirs]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        # Record list of logs to be processed
        logListFilename = self.directory+"/test_summary.txt"        
        try:
            os.remove(logListFilename)      # Remove old file
        except OSError:
            pass

        results = []
        for subdir in self.subdirs:
            sdir = os.path.normpath(str(subdir) + "/post_processed")
            nppPrint("   " + sdir)

            ### Compute RMS ##################################################
            if self.computeRMS:
                passRMS = 0
                if self.log.load(sdir):
                    # Compute and output RMS Report
                    self.log.calculateRMS()
                    passRMS = self.log.printRMSReport()
                    if passRMS == 1:
                        results.append("[PASSED] " + sdir)
                        self.rmsPassResults.append(sdir)
                    else:
                        results.append("[FAILED] " + sdir)
                        self.rmsFailResults.append(sdir)
                else:
                    results.append("[NODATA] " + sdir)
            else:
                results.append("[      ] " + sdir)
            ### Compute RMS ##################################################
        with open(logListFilename, "w") as f:
            f.write("\n".join(results))

        print('-------------------------------------------------------------')
        print(os.path.basename(logListFilename))
        self.print_file_contents(logListFilename)
        print('-------------------------------------------------------------')

    def run_log_folder(self, folder, config_serials):
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
        # log.calculateRMS()
        #
        # self._key_lock.acquire()
        # passRMS = log.printRMSReport()
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

    # 2nd argument: Log Directory
    if len(sys.argv) >= 2:
        directory = sys.argv[1]

    # Debug
    # directory = '/home/walt/src/IS-src/scripts/../../goldenlogs/AHRS'
    # directory = os.path.join('C:/','_IS','goldenlogs','AHRS')
    # directory = os.path.join('C:/','_IS','goldenlogs')
    # directory = 'C:/_IS/goldenlogs'

    # serials = ""
    # directory = 'D:/Dropbox (Inertial Sense)/ISD/logs/202110/20211022_14_NAV_Drive_uins4_branch/20211022_145320'

    if 'directory' not in locals():
        print("First parameter must be directory!")
        exit()

    # 3rd argument: Serial #s
    if len(sys.argv) >= 3:
        serials = sys.argv[2]
    else:
        serials = ['ALL']
        print("Using default value for serials: ", serials)

    # 4th argument: Compute RMS Comparison Report
    if len(sys.argv) >= 4:
        computeRMS = sys.argv[3]
    else:
        computeRMS = 1

    # Run Super NPP
    snpp = SuperNPP(directory, serials, computeRMS=computeRMS)
    snpp.run()

    testSummaryFilename = directory+"/test_summary.txt"
    nppPrint("\n")
    nppPrint("====================  Super NPP Results  ====================")
    print_case(testSummaryFilename, "  RMS Test PASSED:", "[PASSED]")
    print_case(testSummaryFilename, "  RMS Test FAILED:", "[FAILED]")
    print_case(testSummaryFilename, "  Failed to Reprocess:", "[NODATA]")
    nppPrint("=============================================================")

    snpp.exitHack()

