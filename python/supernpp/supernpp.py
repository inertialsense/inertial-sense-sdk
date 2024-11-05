#!/usr/bin/python3

import os
import re
from subprocess import Popen
from os.path import normpath, basename
from threading import Thread
import time
from pathlib import Path
import sys
import threading

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

eprint(Path().resolve())

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

        eprint("=====================  Init SuperNPP  =====================")
        eprint("  Directory: ", self.directory)
        eprint("  config_serials:", self.config_serials)
        eprint("  startMode: ", self.startMode)
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
            if ".sdat" in file or ".dat" in file or ".raw" in file:
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
        logListFilename = directory+"/test_summary.txt"        
        try:
            os.remove(logListFilename)      # Remove old file
        except OSError:
            pass

        f = open(logListFilename, "w")
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
                        f.write("[PASSED] " + sdir + "\n")
                        self.rmsPassResults.append(sdir)
                    else:
                        f.write("[FAILED] " + sdir + "\n")
                        self.rmsFailResults.append(sdir)
                else:
                    f.write("[NODATA] " + sdir + "\n")
            else:
                f.write("[      ] " + sdir + "\n")
            ### Compute RMS ##################################################
        f.close()
        print('-------------------------------------------------------------')
        print(os.path.basename(logListFilename))
        self.print_file_contents(logListFilename)
        print('-------------------------------------------------------------')

    def run_log_folder(self, folder, config_serials):
        # Find the serial numbers in the log
        # subdir = os.path.basename(os.path.normpath(folder))
        (folder, subdir) = os.path.split(folder)

        if config_serials == ["ALL"]:
            serials = []
            for file in os.listdir(os.path.join(folder,subdir)):
                if (".sdat" in file or ".dat" in file or ".raw" in file) and (not "base_station.raw" in file) :
                    ser = int(re.sub('[^0-9]','', file.split("_")[1]))
                    if ser not in serials:
                        serials.append(ser)
        else:
            serials = config_serials

        print("serial numbers")
        print(serials)

        # Determine the log type
        logType = "DAT"
        for file in os.listdir(os.path.join(folder,subdir)):
            if ".sdat" in file:
                logType = "SDAT"
            elif ".dat" in file:
                logType = "DAT"
            elif ".raw" in file:
                logType = "RAW"

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

