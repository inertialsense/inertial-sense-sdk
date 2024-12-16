import argparse
import warnings
import logging
from subprocess import STDOUT

import git
import sys
import os
import re
import contextlib
import socket
import hashlib
import datetime
import functools
from pathlib import Path
import semver  # Use the semantic versioning standard. See: https://semver.org

logging.captureWarnings(True);
logger = logging.getLogger(__name__)

###
#  General purpose utility functions used by all classes
###

def get_value_or_default(default_value, value):
    """"Simple helper function to return a default value of the requested value if None."""
    return default_value if value is None else value


def get_env_or_default(default_value, env):
    """"Simple helper function to get a value from an environment variable,
     or return a default value if the environment variable is not found or set."""
    return get_value_or_default(default_value, os.getenv(env))


def parse_version_and_build_type(ver_string):
    """Attempts to parse and return a tuple of [semver, built-type] from a provided string.
    :param name: A parsable string that looks something like '[v]<M.m.p>[-type][.preRelBuild]
        where <M.m.p> is the Major.minor.patch that describes a version, optionally proceeded by the lowercase letter 'v'
        and were [-type] is an option build-type descriptor of '-rc', '-develop' or '-snap'
        and where [.preRelBuild] is an pre-release build indicator (ie, -rc.5 = "release candidate #5)
        If the [-type] is missing, such that the string looks like 'M.m.p.preRelBuild', then resulting build-type is
        presumed to be a 'release-candidate' (or -rc)
    :return: a tuple of the semantic version object, and the specific build-type string ('rc', 'devel', or 'snap').  If
        no aspect of the string is parsable (ie, an error during parsing) this function returns None
    """

    # strip any leading 'v' before a possible version number
    try:
        ver_string = ver_string[1:] if (ver_string[0] == 'v' and ver_string[1].isdigit()) else ver_string
        version = semver.Version.parse(ver_string)
        type = version.prerelease.split('.')[0] if version.prerelease is not None else None
        if type == 'rc':        # Release Candidate
            rel_type = "'c'"
        elif type == 'alpha':   # Alpha
            rel_type = "'a'"
        elif type == 'beta':    # Beta
            rel_type = "'b'"
        elif type == 'devel':   # Develop
            rel_type = "'d'"
        elif type == 'snap':    # Snap
            rel_type = "'s'"
        else:                   # Release
            rel_type = "0"
    except (ValueError, TypeError):
        version = None
        rel_type = None

    return [version, rel_type]


def get_latest_version(version_list):
    """returns the version from version_list which represents the latest version. This will ignore NoneType."""
    def compare_versions(a, b):
        return a.compare(b)

    filtered = [ v for v in version_list if v is not None ]
    sorted_ver = sorted(filtered, key=functools.cmp_to_key(compare_versions))
    greatest_ver = sorted_ver[-1]
    return greatest_ver


def writer(fn):
    """Intelligently detects asks to write to STDOUT and does so with `with`"""
    @contextlib.contextmanager
    def stdout():
        yield sys.stdout
    return stdout() if fn == "STDOUT" else open(fn, 'w', encoding="utf-8")


class RepositoryInfo:
    """Generic class for representing Git Repository info, but includes semantic version
    and handles some convenience functions for Inertial Sense purposes.
    """
    def __init__(self, base_path, recurse_up=True):
        self.name = None
        self.branch = None
        self.commit = None
        self.repo_description = None
        self.tag = None
        self.distance_from_tag = None
        self.is_dirty = None
        self.version = None
        self.release_type = None # "'!'" # Broken incase nothing else was obvious

        self.git_cmd = git.Git(base_path)
        self.repo = self.find_root_repo(base_path, recurse_up)
        self.working_tree_dir = base_path if self.repo is None else self.repo.working_tree_dir
        self.version = self.get_repo_semver()

        self.get_repo_info()
        return

    def __str__(self):  # Prints human-readable output
        out = f'Repository "{self.name}" ({self.working_tree_dir})'
        out += f'' if self.branch is None else f' on branch "{self.branch}"'
        out += f'' if self.commit is None else f' at commit {str(self.commit)[:8]}'
        out += f'' if self.repo_description is None else f' described as {self.repo_description}'
        out += f' targeting version {self.version}'

        return out

    def write_to_file(self, file=None, prefix=None, dry_run = False):
        """Writes a repository info file to the provided location
        :param file: File to write the repository info to
        :param prefix: a prefix to be appended to each definition in the file
        :param dry_run: if true, the output is sent to STDOUT instead of writing to file
        """
        prefix = '' if prefix is None else prefix
        file = "version/repo_info.h" if file is None else file
        print(f"Writing Repository Info file to {file}{' (DRY-RUN)' if dry_run is True else ''}")
        if dry_run is True:
            file = "STDOUT"

        with writer(file) as f:
            f.write(f"#define {prefix}REPO_NAME \"{self.name}\"\n")
            f.write(f"#define {prefix}REPO_BRANCH \"{self.branch}\"\n")
            f.write(f"#define {prefix}REPO_DESCRIPTION \"{self.repo_description}\"\n")
            f.write(f"#define {prefix}REPO_GIT_COMMIT 0x{self.commit[:8]}\n")
            f.write(f"#define {prefix}REPO_VERSION_NO_META \"{str(self.version).split('+')[0]}\"\n")
            f.write(f"#define {prefix}REPO_VERSION \"{str(self.version)}\"\n")
            f.write(f"#define {prefix}REPO_VERSION_RELEASE_TYPE {str(self.release_type)}\n")
            f.write(f"#define {prefix}REPO_VERSION_MAJOR {str(self.version.major)}\n")
            f.write(f"#define {prefix}REPO_VERSION_MINOR {str(self.version.minor)}\n")
            f.write(f"#define {prefix}REPO_VERSION_REVIS {str(self.version.patch)}\n")
            f.write(f"#define {prefix}REPO_VERSION_PRERELEASE {self.get_prerelease_number()}\n")
            f.write(f"#define {prefix}NSIS_VERSION_NUMBER "
                    f"\"{self.version.major}.{self.version.minor}.{self.version.patch}.{self.get_prerelease_number()}\"\n")
        if file != "STDOUT":
            with open(file, "r", encoding="utf-8") as f:
                print(f.read())


    def get_repo_info(self, fetch_tags=True):
        """Returns a dictionary of the essential repo meta-data for the specified repo.

        The returned dictionary includes the following elements:
            name - the name of the repo, usually the name of the root folder without any path or extention information (but depends on how the repo was originally created)
            repo_description - the results of a `git describe` command, as documented here https://git-scm.com/docs/git-describe
            current branch - the current branch name, or git hash if there is no current commit
            latest commit - the full commit hash for the most recent commit to this repo
            nearest tag
            distance to tag
            dirty

        :type fetchTags: If true, a fetch/pull of all current tags from the remote will be performed first (to provide the most accurate description)
        :return: a dictionary of essential repo metadata
        """

        # if self.repo is None or self.git_cmd is None:
        #     raise TypeError("You must set Repo manually or call get_root_repo() first")

        if fetch_tags == True:
            self.pull_git_tags()

        self.version = self.get_repo_semver()

        try:
            self.name = get_env_or_default(">>INVALID<<" if self.repo is None else self.repo.remotes[0].url.split('.git')[0].split('/')[-1], "REPO_NAME")
        except TypeError:
            self.name = get_value_or_default(self.name, "REPO_NAME")

        try:
            self.branch = get_env_or_default(">>INVALID<<" if self.repo is None else self.repo.active_branch.name, "REPO_BRANCH")
        except TypeError:
            self.branch = get_env_or_default("DETACHED HEAD", "REPO_BRANCH")

        self.commit = get_env_or_default("DEADDEAD" if self.repo is None else str(self.repo.head.commit), "REPO_COMMIT")

        self.repo_description = get_env_or_default(None if self.repo is None else self.git_cmd.describe("--tags"), "REPO_DESCRIPTION")

        self.tag = get_env_or_default("0.0.0" if self.repo is None else self.git_cmd.describe(["--tags", "--abbrev=0"]), "REPO_TAG")

        try:
            self.distance_from_tag = int(get_env_or_default(None if self.repo is None else self.git_cmd.rev_list(["--count", self.tag + "..HEAD"]), "REPO_DISTANCE"))
        except TypeError:
            self.distance_from_tag = None

        self.is_dirty = bool(int(get_env_or_default(True if self.repo is None else self.repo.is_dirty(), "REPO_DIRTY")))

        # self.release_type = get_env_or_default(self.update_release_info(), "REPO_REL_TYPE")

        return {'name': self.name, 'branch': self.branch, 'commit': self.commit, 'description': self.repo_description, 'tag_name': self.tag, 'tag_distance': self.distance_from_tag, 'is_dirty': self.is_dirty, 'version': self.version}


    def get_latest_version(self, distance=None, branch_name=None, target_version=None, prerelease_branch=None, release_tag=None):
        """
        Calculates the latest semantic version for this repo, regardless of tag-name, branch-name, semver-file, etc
        :return: a semantic version
        """
        tag = release_tag if release_tag is not None else self.tag
        # prerelease_branch = prerelease_branch if prerelease_branch is not None else self.tag
        branch_name = branch_name if branch_name is not None else self.branch
        distance = distance if distance is not None else self.distance_from_tag
        rel_type = None

        # parse out each of the specific versions
        latest_release_ver = self.get_latest_release()[1] if tag is None else parse_version_and_build_type(tag)[0]                  # oldest possible version
        latest_rc_ver = self.get_latest_prerelease()[1] if prerelease_branch is None else parse_version_and_build_type(prerelease_branch)[0]   # next oldest
        latest_repo_ver = self.version if target_version is None else parse_version_and_build_type(target_version)[0]                         # least oldest (newest?)

        # check if branch looks like a pre-release branch name
        branch_ver, branch_type = parse_version_and_build_type(branch_name)
        if branch_type is not None and branch_type != 'd' and branch_type != 's':
            latest_rc_ver = latest_rc_ver if latest_rc_ver > branch_ver else branch_ver

        # get the greatest of all the versions
        latest_ver = get_latest_version([latest_release_ver, latest_rc_ver, latest_repo_ver])

        if distance is None or distance == 0 or branch_name == "main":
            # TODO: NOTE that if this build is a non-committed copy/clone of a release commit (ie, it matches any release-tag commit)
            #   or the branch is actually 'main', then the version is ALWAYS determined from a "release tag" name, possibly overriding any
            #   version derived from the local branch name (ie, 'myversion-rc') or from the branches semver file.  In other words:
            #   It doesn't matter what you change the semver or branch name to, if you haven't committed, the version is the release version!
            # This is an "Official Release" (it is the same commit as our release tag), so we get the build-type from the tag name
            [latest_ver, latest_ver_type] = parse_version_and_build_type(tag)
        else:
            latest_ver = self.do_version_bump(latest_ver, latest_rc_ver, latest_release_ver, branch_name)
            if branch_type == "'d'" or branch_name == "develop":    # Develop
                rel_type = 'devel'
            elif branch_type == "'a'":                              # Alpha
                rel_type = 'alpha'
            elif branch_type == "'b'":                              # Beta
                rel_type = 'beta'
            elif branch_type == "'c'":                              # Release Candidate
                rel_type = 'rc'
            else:                                                   # Everything else is a Snapshot
                rel_type = 'snap'
                latest_ver = latest_ver.replace(prerelease=f'snap.{distance}')

        latest_ver = latest_ver.replace(prerelease=None) if rel_type is None else latest_ver.replace(prerelease=f'{rel_type}.{distance}' if distance > 0 else f'{rel_type}')
        return latest_ver



    def get_latest_release(self):
        """
        Digs through repository release-tags looking for the last full release (where semver.prerelease == None)
        :return: the semantic version of the last release
        """
        release_tag = self.tag  # Recurse backwards until last release tag is found
        try:
            [release_tag_ver, release_tag_build] = parse_version_and_build_type(release_tag)
            while release_tag_ver is not None and release_tag_ver.prerelease is not None:
                release_tag = get_env_or_default( self.git_cmd.describe(["--tags", "--abbrev=0", release_tag+"^"]), "REPO_RELEASE_TAG" )
                [release_tag_ver, release_tag_build] = parse_version_and_build_type(release_tag)  # This should get the latest release
        except ValueError:
            warnings.warn(f'A tag between you and the closest release tag to your current HEAD is not formatted properly\n'
                          f'Please check the following tags: {self.tag}, {release_tag}\n'
                          f'Going to default version {str(self.version)}')
            release_tag_ver = self.version

        return [release_tag, release_tag_ver]


    def get_latest_prerelease(self, branch_pattern=r'\d.\d.\d-(alpha|beta|rc|devel|snap)$'):
        """Digs through remote branches, looking for matching branch-name patterns (ie, '2.3.5-rc'), parsing
        their names to extract a version, and returns the latest/greatest branch name and version.
        :return: a tuple of the branch name and the semantic version of the last pre-release branch
        """

        latest_branch = None
        latest_version = semver.Version.parse('0.0.0')

        if self.repo is not None:
            for branch in self.repo.remotes[0].refs:
                if branch is not None and re.match(branch_pattern, branch.remote_head):
                    [version, type] = parse_version_and_build_type(branch.remote_head)
                    if version and version >= latest_version:
                        latest_version = version
                        latest_branch = branch.name

        return [latest_branch, latest_version]


    def find_root_repo(self, path=os.path.dirname(os.path.realpath(__file__)), recurse_up=True, max_depth=10):
        """ Attempts to locate the root repo for the given path (by default, the location of this scripts).
        If 'recurse_up' is True, if the repo is a submodule, it will locate the parent repo, ascending up until
        it reaches the top-most repo or max_depth is reached. If 'recurse_up' is False, it will return the
        nearest repo which contains the specified path.
        :returns: the repository object for the identified root
        """

        while True:
            try:
                self.repo = git.Repo(path, search_parent_directories=True)
                self.git_cmd = git.Git(self.repo.working_tree_dir)
                path = self.git_cmd.rev_parse("--show-superproject-working-tree")

                if not recurse_up or (max_depth < 0) or (path == ""):
                    return self.repo

                max_depth = max_depth - 1

            except git.NoSuchPathError as err:
                warnings.warn(f"Specified path '{path}' references in invalid or corrupt git repository (submodule, etc).")
                self.name = "BROKEN_GIT!"       # an indicator that there is something wrong with this git repo
                self.repo = None
                return self.repo


    def update_release_info(self, build_info=None):
        """ Determines the release type based on the repository information (branch, tag, dirty, etc) and updates
        the 'release_type' property with the release-type, and the 'version' property. The 'version' is updated to
        the fully-formed Semantic Version, including build information.
        :return: the release-type as one of 'r' (release), 'a' (alpha), 'b' (beta), 'c' (candidate), 'd' (develop), or 's' (snapshot)
        """

        self.version = self.get_latest_version()
        if self.branch == "develop":                                      # Develop
            self.release_type = "'d'"
        elif re.match(r"\d.\d.\d-rc", self.branch) is not None:  # Release Candidate
            self.release_type = "'c'"
        elif re.match(r"\d.\d.\d-b", self.branch) is not None:   # Beta
            self.release_type = "'b'"
        elif re.match(r"\d.\d.\d-a", self.branch) is not None:   # Alpha
            self.release_type = "'a'"
        else:                                                          # Snapshot
            self.release_type = "'s'"


        if build_info:
            self.version = self.version.replace(build = f'{self.commit[:8]}{ "-dirty" if self.is_dirty else "" }.{build_info.hostkey}-{build_info.build_number}')

        return self.release_type


    def do_version_bump(self, latest_ver=None, latest_rc=None, latest_release=None, branch=None):
        """
        WARNING:: DO NOT MODIFY THIS FUNCTION UNLESS YOU ARE ABSOLUTELY SURE YOU KNOW WHAT YOU ARE DOING -- AND WHY!!!

        This function attempts to determine if the current repo-version needs to be bumped to a new version, and if so, HOW it should be
           bumped (ie, major-bump, minor-bump, patch-bump, etc).  The rules here can be janky and fickle, and sometimes they require
           advanced knowledge of the repo structure & git history, and sometimes they can rely on a particular branch/tag naming
           convention, and other "easily-human-breakable" things.  If you are getting weird versions, this is probably the place to
           start your investigation.

        :param latest_ver: this indicates the "target" version which this branch/commit-chain attains to be; it is merely a suggestion.
                If this is set to None (default), it will be read from the semver file in the root of the current repo. This version will
                always be updated to be EQUAL to the greater of latest_rc or latest_release (It cannot be lesser than either)
        :param latest_rc: this is the latest release-candidate version.
                If this is set to None (default), it is derived from parsing all repo branches that look like "#.#.#-rc", and extracting
                a version from the name, and using the latest.
        :param latest_release: this is the latest version for a full release
                If this is set to None (default), it is derived from parsing all the repo tags that look like v#.#.# or #.#.#, extracting
                a version from the tag name, and using the latest.

        :param branch: this is the name of the current branch. Some rules make determinations based on the branch name.
        :return: the newly updated semantic version object.
        """
        latest_ver = latest_ver if latest_ver is not None else self.version
        latest_rc = latest_rc if latest_rc is not None else self.get_latest_release_candidate()[1]
        latest_release = latest_release if latest_release is not None else self.get_latest_release()[1]
        branch = branch if branch is not None else self.branch


        if branch == 'develop' and (latest_ver <= latest_rc or latest_ver <= latest_release):
            # if building on develop and the release-version is the same as the develop-version, then minor-bump
            # the develop-version (because latest_ver )
            # last release tag is 2.3.0, build on develop SHOULD be on 2.4.0 (so, we'll make it that way)
            latest_ver = latest_release.bump_minor()
        else:
            if (latest_ver <= latest_release) and (latest_rc <= latest_release):
                # we will ALWAYS bump the patch version if the latest RC is BEHIND the latest release
                #    AND our target version is less than or equal to the latest release (ie, we haven't
                #    staged a newer release yet)
                # -- this suggests that the next release will most likely be a patch of the latest release
                # -- Additionally, we can NEVER create a version for a release (candidate or otherwise) that
                #    is prior to a release of the same version.
                #
                #    Remember that 1.2.0-rc IS A LESSER release (a pre-release) to 1.2.0
                latest_ver = latest_ver.bump_patch()
            else:
                # otherwise, it suggests that there is a release-candidate "in-progress".  We GENERALLY
                #    won't have a release 2.3.0 while ALSO having a release-candidate 2.4.5-rc (which
                #    suggests that there VERY LIKELY exists a release-2.4.4).  If a candidate is "in-
                #    progress", then we don't need to bump anything, the current branch is also the RC
                #    branch.
                pass

        # branch_ver, branch_type = parse_version_and_build_type(branch)
        # if (branch_ver is not None) and (branch_type != ""):

        # I've created "my_branch" from 'develop' and I build... I should build with the last-release + 1 minor version (but as -snap)
        #   - this SHOULD use the semver file as the latest version
        #      if semver <= last_release_ver  let's

        # I've created "my_branch" from '2.4.7-rc' and I build... I should build with release-candidate version (but as -snap)
        #   - this SHOULD use the semver file as the latest version
        #      if semver <= last_release_ver increment patch-version

        return latest_ver


    def pull_git_tags(self):
        """Fetches the specified git repo and force updates of all git tags from the remote."""

        # if self.repo is None or self.git_cmd is None:
        #    raise TypeError("You must set Repo manually or call get_root_repo() first")

        logger.info("Pulling latest git tag information, if the resource requires additional authentication provide it now.\n")
        try:
            self.git_cmd.pull(["--tags", "-f"])
        except git.exc.GitCommandError:
            warnings.warn("Failed to acquire git tag information!\n"
                          "You likely don't have internet access or just forgot to authenticate.\n"
                          "Continuing anyways... Data may be out of date.")


    def get_repo_semver(self, version=semver.Version.parse('0.0.0')):
        """Determines the semantic version for the specified repo, reading a semver file in the repo root
        or return. Relies on a semver file to exist at the root/working directory of the repo.
        """

        repo_path = Path(self.working_tree_dir) / 'semver'
        try:
            with open(repo_path, 'r') as file:
                verstr = file.read().replace('\n', '')
                parsed_version = semver.Version.parse(verstr)
                version = parsed_version if parsed_version >= version else version
        except FileNotFoundError:
            version = semver.Version.parse("0.0.0-snap.0+BROKEN.GIT")

        return version


    def get_prerelease_number(self, version=None):
        """Returns the numerical value (as an int) of the prerelease field of a semantic version
        or 0, if its undefined, or unparsable."""
        version = version if version is not None else self.version
        if version is not None and version.prerelease is not None:
            parts = version.prerelease.split('.')
            if len(parts) > 0:
                try:
                    return int(parts[-1])
                except ValueError:
                    return 0

        return  0


    def get_latest_merge(self, branch=None, target='main'):
        """Attempts to locate the last time 'branch' was merged into by another branch"""
        if self.repo is None or len(self.repo.remotes) == 0:
            return None

        self.repo.remotes[0].fetch()
        branch_head = self.repo.remotes[0].refs[branch]

        if branch_head is None:
            logger.error(f"Attempt to located the most recent merge of {branch} into {target} returned no results.")
            return None

        commit = branch_head.commit
        while True:
            print(f"{commit.hexsha} [{commit.author}, {commit.authored_datetime}] : {commit.message.rstrip()}")
            if len(commit.parents) != 1:
                break
            commit = commit.parents[0]


class BuildInfo:
    """Generic class for representing current build info, and handles some convenience
    functions for Inertial Sense purposes.
    """
    def __init__(self, repoInfo=None, build_info_file=None):
        self.repo_info = repoInfo

        # Default Build Info Values
        self.build_number = self.get_build_number(build_info_file) if build_info_file is not None else 0
        self.hostname = self.get_hostname("unknown-host")
        self.hostkey = self.get_hostkey(self.hostname)
        self.timestamp = datetime.datetime.now(datetime.timezone.utc)  # Use UTC time here to prevent time travel

        return


    def __str__(self):  # Prints human-readable output
        out = f'Building version {str(self.repo.version)}'
        out += f' from repo {self.repo_info.name}'
        out += f' on branch {self.repo_info.branch}'
        out += f' at commit {str(self.repo_info.commit)[:8]}'
        out += f' on host {self.build_info.hostname}'
        out += f' at time {str(self.build_info.build_time)}\n'
        out += f'Build number: {self.build_info.build_number:d}'
        out += f'Your Host key: 0x{self.build_info.hostkey}'
        out += f'Combined build ID: {self.build_info.hostkey}.{self.build_info.build_number:d}\n'
        return out


    def write_to_file(self, file=None, prefix=None, dry_run=False):
        """Writes a build info file (C-language header file) to the provided location
        :param file: File to write the repository info to
        :param prefix: a prefix to be appended to each definition in the file
        :param dry_run: if true, the output is sent to STDOUT instead of writing to file
        """
        prefix = '' if prefix is None else prefix
        file = BuildInfoFileLocation if file is None else file
        print(f"Writing Build Info file to {file}{' (DRY-RUN)' if dry_run is True else ''}")

        if dry_run is True:
            file = "STDOUT"

        with writer(file) as f:
            f.write(f"#define {prefix}BUILD_NUMBER {self.build_number:d}\n")
            f.write(f"#define {prefix}BUILD_TYPE {self.repo_info.release_type}\n")
            f.write(f"#define {prefix}BUILD_HOSTNAME \"{self.hostname}\"\n")
            f.write(f"#define {prefix}BUILD_HOST_KEY 0x{self.hostkey}\n")
            f.write(f"#define {prefix}BUILD_DATE_YEAR {self.timestamp.year}\n")
            f.write(f"#define {prefix}BUILD_DATE_MONTH {self.timestamp.month}\n")
            f.write(f"#define {prefix}BUILD_DATE_DAY {self.timestamp.day}\n")
            f.write(f"#define {prefix}BUILD_DATE \"{self.timestamp.date()}\"\n")
            f.write(f"#define {prefix}BUILD_TIME_HOUR {self.timestamp.hour}\n")
            f.write(f"#define {prefix}BUILD_TIME_MINUTE {self.timestamp.minute}\n")
            f.write(f"#define {prefix}BUILD_TIME_SECOND {self.timestamp.second}\n")
            f.write(f"#define {prefix}BUILD_TIME_MILLISECOND {int(self.timestamp.microsecond / 10000):d}\n")
            f.write(f"#define {prefix}BUILD_TIME \"{self.timestamp.strftime("%H%M%S")}\"\n")
        if file != "STDOUT":
            with open(file, "r", encoding="utf-8") as f:
                print(f.read())


    def get_build_number(self, file=None, default_build_num=0):
        """Returns the next build number for this repo, determined by an overriding environment value "BUILD_NUMER",
        or read from the specified srcFile.  Is srcFile is None, and no environment variable is set, this function
        will raise an exception
        :param file:
        :return:
        """

        build_number = None
        build_number_env = os.getenv("BUILD_NUMBER")
        if build_number_env is not None:
            build_number = int(build_number_env) + 1
        else:
            try:
                parsed_build_num = None
                with open(file, "r", encoding="utf-8") as f:
                    # simple parser to look for:  #define BUILD_NUMBER <buildnum>
                    for line in f:
                        statement = line.strip("\n").split()
                        try:
                            if statement[0].upper() == "#DEFINE":
                                if statement[1] == "BUILD_NUMBER":
                                    parsed_build_num = int(statement[2]) + 1
                        except IndexError:
                            continue

                    if parsed_build_num:
                        build_number = parsed_build_num
            except FileNotFoundError:
                print("Build info file not found, assuming first build")

        return build_number


    def get_hostname(self, default_hostname="unknown-host"):  # Gets the system hostname
        """the hostname of this machine, unless overridden by the 'BUILD_HOSTNAME' environment variable"""
        self.hostname = get_env_or_default(get_value_or_default(default_hostname, socket.gethostname()), "BUILD_HOSTNAME")
        return self.hostname


    def get_hostkey(self, hostname=None):  # Generates a 5 charactor hex string for the system hostname
        """returns a partial hash of the hostname; essentially a mechanism to encode a form of unique host-id into a handful of bits"""
        hostname = hostname if hostname is not None else self.hostname
        return get_env_or_default(hashlib.md5(bytes(hostname, "utf-8")).hexdigest()[:5], "BUILD_HOSTKEY")


class ISVersionTools:
    def __init__(self, base_path, recurseUp=True):

        self.ScriptDir = os.path.dirname(os.path.realpath(__file__))
        self.basePath = None
        self.buildInfoFileLocation = None
        self.repositoryInfoFileLocation = None
        self.zephyrVersionFileLocation = None
        self.has_zephyr = False
        self.dry_run = False

        self.repo = RepositoryInfo(base_path, recurseUp)
        self.build = BuildInfo(self.repo)
        self.file_paths = self.set_default_file_paths(base_path)
        return


    def update_build_info(self):
        """
        Updates Build information; DOES NOT WRITE anything
        :return:
        """
        return

    def update_version_info(self):
        """
        Updates/Generates Version information from Repo/Build info; DOES NOT WRITE anything
        :return:
        """
        return

    def write_all_info(self, prefix=None, write_path=None, generate_zephyr_version=False, dry_run=False):
        """
        Generates updated repoInfo.h, buildInfo.h, version.h, and optionally a zephyr VERSION file.
        :param write_path:
        :param generate_zephyr_version:
        """
        self.generate_version()

        if prefix is None:
            if self.repo.name == "inertial-sense-sdk":
                prefix = 'IS_SDK_'
            elif self.repo.name == "imx":
                prefix = ''
            elif self.repo.name == "is-gpx":
                prefix = 'IS_GPX_'
            else:
                prefix = ''

        base_path = Path(write_path if write_path is not None else ".")
        self.repo.write_to_file(str(base_path / self.repositoryInfoFileLocation), prefix, dry_run)
        self.build.write_to_file(str(base_path / self.buildInfoFileLocation), prefix, dry_run)
        if generate_zephyr_version is True and self.has_zephyr:
            self.write_zephyr_version_file(str(base_path / self.zephyrVersionFileLocation), dry_run=dry_run)


    def generate_version(self):
        """Generates updated version info from repo_info and build_info dictionaries, specifically Tag, Dirty, DistanceFromTag, Branch, HostKey, BuildNumber
        :return: the fully-filled semantic version, including release-type and build info
        """
        self.repo.update_release_info(self.build)        # update the release version and type based on the latest repo info, etc
        return self.repo.version

    def version(self):
        if self.repo is None:
            return None

        if self.repo.version is None:
            self.repo.update_release_info(self.build)

        return self.repo.version

    def repo_info(self):
        return self.repo

    def build_info(self):
        return self.build


    def set_default_file_paths(self, path=None, zephyr_path=None, dry_run=False):
        """Sets default file paths. Path is a path to version.h"""

        self.dry_run = dry_run or (str(os.getenv("DRYRUN")).lower() == "y")

        # check if this repo is a zephyr project (look for prj.conf and other indicators)
        self.zephyr_dotwest = Path(self.repo.working_tree_dir) / ".west"
        self.zephyr_prjConf = Path(self.repo.working_tree_dir) / "prj.conf"
        self.zephyr_sysbuildConf = Path(self.repo.working_tree_dir) / "sysbuild.conf"
        self.zephyr_zephyrDir = Path(self.repo.working_tree_dir) / "zephyr"

        if self.zephyr_dotwest.exists() or self.zephyr_zephyrDir.exists() or self.zephyr_prjConf.exists() or self.zephyr_sysbuildConf.exists():
            zephyr_path = self.repo.working_tree_dir if zephyr_path is None else zephyr_path

        self.has_zephyr = (zephyr_path is not None)

        if path is None:
            path = "./version"

        self.basePath = path
        self.buildInfoFileLocation = self.basePath + "/buildInfo.h"
        self.repositoryInfoFileLocation = self.basePath + "/repositoryInfo.h"
        self.zephyrVersionFileLocation = None if zephyr_path is None else zephyr_path + "/VERSION"
        if dry_run is True:
            self.buildInfoFileLocation = "STDOUT"
            self.repositoryInfoFileLocation = "STDOUT"
            self.zephyrVersionFileLocation = None if zephyr_path is None else "STDOUT"

        self.file_paths = { 'basePath': self.basePath, 'buildInfoFile': self.buildInfoFileLocation, 'repoInfoFile': self.repositoryInfoFileLocation, 'zephyrVersionFile': self.zephyrVersionFileLocation }
        return self.file_paths


    def write_zephyr_version_file(self, file=None, version=None, dry_run=False):
        """Writes a Zephyr VERSION file to the provided location"""

        version = version if version is not None else self.repo.version
        file = ZephyrVersionFileLocation if file is None else file
        print(f"Writing Zephyr Version File to {file}{'(DRY-RUN)' if dry_run is True else ''}")

        if dry_run is True:
            file = "STDOUT"

        with writer(file) as f:
            f.write(f"VERSION_MAJOR = {str(version.major)}\n")
            f.write(f"VERSION_MINOR = {str(version.minor)}\n")
            f.write(f"PATCHLEVEL = {str(version.patch)}\n")
            f.write(f"VERSION_TWEAK = {self.repo.get_prerelease_number(version)}\n")
            f.write(f"EXTRAVERSION = {str(version.prerelease)}\n")
        if file != "STDOUT":
            with open(file, "r", encoding="utf-8") as f:
                print(f.read())



def test_version(actual, expected, fail_msg = None):
    """Simple comparison of an actual and expection version string, with an optional message if they fail to match"""
    if isinstance(actual, str):
        actual = semver.Version.parse(actual)

    if isinstance(expected, str):
        ver2 = semver.Version.parse(expected)

    if actual != expected:
        print(f"[FAIL] {actual} != {expected} <== {fail_msg}")
    else:
        print(f"[PASS] {actual} == {expected}")


def run_version_tests():
    """A series of tests comparing different versioning conditions and their expected results"""
    repo = RepositoryInfo('.', False)
    # All of the following should return the same as the release-tag; because they are all distance=0, they are ALL the same as 'main' which is the last release
    test_version(repo.get_latest_version( 0,   "SN-1234",     "1.0.0", "0.0.99-rc",   "0.0.1"),
                 "0.0.1")        # Version == 0.0.1
    test_version(repo.get_latest_version( 0,   "develop",     "1.0.0", "0.0.99-rc",   "0.0.12"),
                 "0.0.12")       # Version == 0.0.12
    test_version(repo.get_latest_version( 0,   "1.0.0-rc",    "1.0.0", "0.0.99-rc",   "0.0.25"),
                 "0.0.25")      # Version == 0.0.25
    test_version(repo.get_latest_version( 0,   "main",        "1.0.0", "0.0.99-rc",   "1.0.0"),
                 "1.0.0")           # Version == 1.0.0
    print()

    test_version(repo.get_latest_version( 1,   "SN-1234",     "1.0.0", "0.0.99-rc",   "0.0.1"),
                 "1.0.0-snap.1",    "because target is 1.0.0 (greatest), and is a snapshot branch")
    test_version(repo.get_latest_version( 7,   "develop",     "1.0.0", "0.0.99-rc",   "0.0.12"),
                 "1.0.0-devel.7",   "because target is greatest (1.0.0), and its the develop branch")
    test_version(repo.get_latest_version( 15,  "1.0.0-alpha", "1.0.0", "0.0.99-rc",   "0.0.25"),
                 "1.0.0-alpha.15",  "because target is 1.0.0 (greatest), and its a pre-release branch")
    test_version(repo.get_latest_version( 21,  "1.0.0-rc",    "1.0.0", "0.0.99-rc",   "0.0.25"),
                 "1.0.0-rc.21",     "because we haven't released 1.0.0, and its a pre-release")
    test_version(repo.get_latest_version( 31,  "main",        "1.0.0", "0.0.99-rc",   "1.0.0"),
                 "1.0.0",           "because we finally released 1.0.0; distance doesn't matter")
    print()

    test_version(repo.get_latest_version( 3,   "SN-1234",     "1.0.0", "0.0.99-rc",   "1.0.0"),
                 "1.0.1-snap.3",    "because we RELEASED 1.0.0, we haven't created a new pre-release branch, and its a snapshot branch; it should have patch bumped the target version")
    test_version(repo.get_latest_version( 85,  "develop",     "1.0.0", "0.0.99-rc",   "1.0.0"),
                 "1.1.0-devel.85",  "because we RELEASED 1.0.0, and the branch is 'develop'")
    test_version(repo.get_latest_version( 14,  "1.0.0-alpha", "1.0.0", "1.0.0-alpha", "1.0.0"),
                 "1.0.1-alpha.14",  "because we RELEASED 1.0.0, so we force the version to patch-bump to 1.0.1")
    test_version(repo.get_latest_version( 33,  "1.0.1-rc",    "1.0.0", "1.0.1-rc",    "1.0.1"),
                 "1.0.2-rc.33",     "because we RELEASED 1.0.1, and the branch is the pre-release branch, which is older")
    test_version(repo.get_latest_version( 42,  "main",        "1.0.0", "1.0.2-rc",    "1.0.1"),
                 "1.0.1",           "because this is 'main', and our last release was 1.0.1; nothing else matters")
    print()

    test_version(repo.get_latest_version( 185, "SN-1234",     "1.3.0", "1.2.4-rc",    "1.2.4"),
                 "1.3.0-snap.185",  "because branch is 'snapshot', and the target version (1.3.0) is greater than pre-release or release")
    test_version(repo.get_latest_version( 18,  "develop",     "1.3.0", "1.1.99-rc",   "1.2.0"),
                 "1.3.0-devel.18",  "because the branch is 'develop' and the target version is 1.3.0 which is greater than pre-release or release")
    test_version(repo.get_latest_version( 47,  "1.3.0-beta",  "2.0.0", "1.2.0-rc.5",  "1.2.0"),
                 "2.0.0-beta.47",   "because this is a pre-release branch, but the target ver is 2.0.0 which is greatest")
    test_version(repo.get_latest_version( 73,  "1.3.1-rc",    "1.0.0", "1.2.3-rc",    "1.2.2"),
                 "1.3.1-rc.73",     "because release 1.0.0 is OLD, and this branch is a pre-release branch - it should have parsed the branch name for the version")
    test_version(repo.get_latest_version( 152, "main",        "1.3.4", "1.3.5-rc",    "1.3.5"),
                 "1.3.5",           "because this is 'main', and our last release was 1.3.5; nothing else matters")
    print()


def run_repo_tests():
    """A series of tests to validate the RepositoryInfo class"""
    repoInfo = RepositoryInfo("/bogus-directory")
    repoInfo.update_release_info()
    print(repoInfo)


def run_tests():
    """Performs all the tests for this module"""
    run_version_tests()
    run_repo_tests()


def main():  # Main function to use if if not used as a library
    parser = argparse.ArgumentParser("version_tools", "Utility application to generate version/release information from the git repo (both local & remote).")
    parser.add_argument('base_dir')
    parser.add_argument('-p', '--prefix', type=str)
    parser.add_argument('-r', '--recurse', default=False, action='store_true')
    parser.add_argument('-d', '--dry-run', default=False, action='store_true')
    parser.add_argument('-z', '--zephyr', default=False, action='store_true')

    args = parser.parse_args()
    print()
    print(f"Parsed arguments: {args}, Current Directory: {os.path.realpath(os.curdir)}")

    base_dir = args.base_dir if args.base_dir is not None else os.curdir
    print(f'Using directory "{base_dir}" for repository root.')
    versionInfo = ISVersionTools(base_dir, False)
    print(versionInfo.repo)
    print()

    myver = versionInfo.generate_version()
    print(myver)
    print()

    versionInfo.write_all_info(args.prefix, write_path=None, generate_zephyr_version=args.zephyr, dry_run=args.dry_run)



if __name__ == "__main__":  # If we are the running script file call main()
    main()
    # run_tests()
