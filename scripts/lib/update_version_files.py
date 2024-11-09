import warnings
import git
import sys
import os
import re
import contextlib
import socket
import hashlib
import datetime
import semver  # Use the semantic versioning standard. See: https://semver.org

# Default Global Module Variables
ScriptDir = os.path.dirname(os.path.realpath(__file__))
Repo = None
BasePath = None
BuildInfoFileLocation = None
RepositoryInfoFileLocation = None
ZephyrVersionFileLocation = None

# Default Repo Values if asking Git doesn't work for some reason
RepoName = "BROKEN-GIT!"
Branch = "BROKEN-GIT!"
Commit = "DEADDEAD"
RepoDescription = "BROKEN-GIT!"
TagName = "0.0.0-snap.0+BROKEN.GIT"
DistanceFromTag = 0
Dirty = True
FullVer = semver.Version.parse("0.0.0-snap.0+BROKEN.GIT")
PreReleaseSplit = ["snap", 0]
PreReleaseType = "snap"
PreReleaseVersion = 0

# Default Build Info Values
BuildNumber = 0
Hostname = "Unknown-Host"
HostKey = "DEAD0"
BuildType = "'!'"  # Broken incase nothing else was obvious

# Get Build Time
Now = datetime.datetime.now(datetime.timezone.utc)  # Use UTC time here to prevent time travel
BuildTime = Now.strftime("%H%M%S")


def get_root_repo(path=ScriptDir):  # Find Root Repo
    repo = git.Repo(path, search_parent_directories=True)
    while True:  # Get the highest level project
        git_cmd = git.Git(repo.working_tree_dir)
        root_repo = git_cmd.rev_parse("--show-superproject-working-tree")
        if root_repo != "":
            repo = git.Repo(root_repo)
        else:
            break
    global Repo
    Repo = repo
    return repo


def pull_git_tags():  # Fetches git repo and force updates git tags from remote
    if Repo is None:
        raise TypeError("You must set Repo manually or call get_root_repo() first")
    git_cmd = git.Git(Repo.working_tree_dir)
    print("Pulling latest git tag information, if the resource requires additional authentication provide it now.\n")
    try:
        git_cmd.pull(["--tags", "-f"])
    except git.exc.GitCommandError:
        warnings.warn("Failed to acquire git tag information!\n"
                      "You likely don't have internet access or just forgot to authenticate.\n"
                      "Continuing anyways... Data may be out of date.")


def set_default_file_paths(path, zephyr_path=None):  # Sets default file paths. Path is a path to version.h
    zephyr_path = Repo.working_tree_dir if zephyr_path is None else zephyr_path
    if path is None:
        raise TypeError("You must specify a path to set_default_file_paths()")
    if zephyr_path is None:
        raise TypeError("You must specify a zephyrPath to set_default_file_paths() or call get_root_repo() first")
    global BasePath, BuildInfoFileLocation, RepositoryInfoFileLocation, ZephyrVersionFileLocation
    BasePath = path
    BuildInfoFileLocation = BasePath + "/buildInfo.h"
    RepositoryInfoFileLocation = BasePath + "/repositoryInfo.h"
    ZephyrVersionFileLocation = zephyr_path + "/VERSION"
    if str(os.getenv("DRYRUN")).lower() == "y":
        BuildInfoFileLocation = "STDOUT"
        RepositoryInfoFileLocation = "STDOUT"
        ZephyrVersionFileLocation = "STDOUT"


def get_value_or_default(default_value, value):
    if value is None:
        return default_value
    return value


def get_env_or_default(default_value, env):
    return get_value_or_default(default_value, os.getenv(env))


def get_repo_info():  # Gets all needed data from a repo
    if Repo is None:
        raise TypeError("You must set Repo manually or call get_root_repo() first")
    git_cmd = git.Git(Repo.working_tree_dir)
    global RepoName, Branch, Commit, RepoDescription, TagName, DistanceFromTag, Dirty
    RepoName = get_env_or_default(Repo.remotes[0].url.split('.git')[0].split('/')[-1], "REPO_NAME")
    try:
        Branch = get_env_or_default(Repo.active_branch.name, "REPO_BRANCH")
    except TypeError:
        Branch = get_env_or_default("DETACHED HEAD", "REPO_BRANCH")
    Commit = get_env_or_default(str(Repo.head.commit), "REPO_COMMIT")
    RepoDescription = get_env_or_default(git_cmd.describe("--tags"), "REPO_DESCRIPTION")
    TagName = get_env_or_default(git_cmd.describe(["--tags", "--abbrev=0"]), "REPO_TAG")
    DistanceFromTag = int(get_env_or_default(git_cmd.rev_list(["--count", TagName + "..HEAD"]), "REPO_DISTANCE"))
    Dirty = bool(int(get_env_or_default(Repo.is_dirty(), "REPO_DIRTY")))
    return {"name": RepoName, "branch": Branch, "commit": Commit, "description": RepoDescription, "tag": TagName,
            "tagDistance": DistanceFromTag, "dirty": Dirty}


def get_build_number(file=None):
    build_number_env = os.getenv("BUILD_NUMBER")
    if build_number_env is None:
        file = BuildInfoFileLocation if file is None else file
        global BuildNumber
        try:
            with open(file, "r", encoding="utf-8") as f:
                for line in f:
                    statement = line.strip("\n").split()
                    try:
                        if statement[0] == "#define":
                            if statement[1] == "BUILD_NUMBER":
                                BuildNumber = int(statement[2]) + 1
                    except IndexError:
                        continue
        except FileNotFoundError:
            print("Build info file not found, assuming first build")
    else:
        BuildNumber = int(build_number_env)
    return BuildNumber


def get_hostname():  # Gets the system hostname
    global Hostname
    Hostname = get_env_or_default(socket.gethostname(), "BUILD_HOSTNAME")
    return Hostname


def get_host_key():  # Generates a 5 charactor hex string for the system hostname
    global HostKey
    HostKey = get_env_or_default(hashlib.md5(bytes(Hostname, "utf-8")).hexdigest()[:5], "BUILD_HOSTKEY")
    return HostKey


def strip_v_from_tag(tag):
    if tag[0] == 'v':
        return tag[1:]
    return tag


def generate_version_data():  # Generates version data from Tag, Dirty, DistanceFromTag, Branch, HostKey, BuildNumber
    global FullVer, PreReleaseSplit, PreReleaseType, PreReleaseVersion, DistanceFromTag
    try:
        last_release_tag = TagName  # Recurse backwards until last release tag is found
        tag_ver = semver.Version.parse(strip_v_from_tag(TagName))
        release_tag_ver = tag_ver
        git_cmd = git.Git(Repo.working_tree_dir)
        while release_tag_ver.prerelease is not None:
            last_release_tag = git_cmd.describe(["--tags", "--abbrev=0", last_release_tag+"^"])
            last_release_tag = get_env_or_default(last_release_tag, "REPO_RELEASE_TAG")
            last_release_tag = strip_v_from_tag(last_release_tag)
            release_tag_ver = semver.Version.parse(last_release_tag)  # This should get the latest release
    except ValueError:
        warnings.warn("A tag between you and the closest release tag to you current HEAD is not formatted properly\n"
                      "Please check the following tags: "+TagName+", "+last_release_tag+"\n"
                      "Going to default version 0.0.0")
        tag_ver = semver.Version.parse('0.0.0')
        release_tag_ver = tag_ver
        DistanceFromTag = 255

    # Set prerelease value automatically if not on a tag
    branchVer = strip_v_from_tag(Branch)
    if not Dirty:
        try:
            if (DistanceFromTag == 0 or Branch == "main"):  # Release
                FullVer = tag_ver
            elif re.fullmatch(r"\d.\d.\d-rc", branchVer) is not None:  # Release Candidate
                FullVer = semver.Version.parse(branchVer)
                FullVer = FullVer.replace(prerelease=f"rc.{DistanceFromTag}")
            elif re.fullmatch(r"\d.\d.\d-b", branchVer) is not None:  # Beta
                FullVer = semver.Version.parse(branchVer)
                FullVer = FullVer.replace(prerelease=f"beta.{DistanceFromTag}")
            elif re.fullmatch(r"\d.\d.\d-a", branchVer) is not None:  # Alpha
                FullVer = semver.Version.parse(branchVer)
                FullVer = FullVer.replace(prerelease=f"alpha.{DistanceFromTag}")
            elif Branch == "develop":  # Devel
                FullVer = release_tag_ver.bump_patch().replace(prerelease=f"devel.{DistanceFromTag}")
            else:  # Snapshot
                FullVer = release_tag_ver.bump_patch().replace(prerelease=f"snap.{DistanceFromTag}")
        except ValueError:
            warnings.warn("Failed to parse version data from tag or branch.\n"
                          "Please check the following tags: "+TagName+", "+last_release_tag+"\n"
                          "Please check the following branch: "+Branch+"\n"
                          "Please ensure the tags are SemVer compliant\n"
                          "If the branch contains an semver string within ensure it is also compliant\n"
                          "Going to default version 0.0.0")
            tag_ver = semver.Version.parse("0.0.0")
            release_tag_ver = tag_ver
            FullVer = tag_ver
            DistanceFromTag = 255
    else:  # Snapshot
        FullVer = release_tag_ver.bump_patch().replace(prerelease=f"snap.{DistanceFromTag}")

    # Parse prerelease value
    if FullVer.prerelease is not None:
        PreReleaseSplit = FullVer.prerelease.split(".")
        PreReleaseType = PreReleaseSplit[0]
        PreReleaseVersion = PreReleaseSplit[1]

    # Create SemVer Build Metadata value
    build_meta = f"{Commit[:8]}"
    if Dirty:
        build_meta = build_meta + "-dirty"
    build_meta = build_meta + f".{HostKey}-{BuildNumber:d}"
    FullVer = FullVer.replace(build=build_meta)
    return FullVer


def compute_build_type():  # Computes the build type from DistanceFromTag, PreReleaseType, Branch, Dirty
    global BuildType
    branchVer = strip_v_from_tag(Branch)
    if DistanceFromTag == 0:
        if PreReleaseType == 'rc':  # Release Candidate
            BuildType = "'c'"
        elif PreReleaseType[0] == 'a':  # Alpha
            BuildType = "'a'"
        elif PreReleaseType[0] == 'b':  # Beta
            BuildType = "'b'"
        else:  # Release
            BuildType = "0"
    elif Branch == "main":  # Release
        BuildType = "0"
    elif re.match(r"\d.\d.\d-rc", branchVer) is not None:  # Release Candidate
        BuildType = "'c'"
    elif re.match(r"\d.\d.\d-b", branchVer) is not None:  # Beta
        BuildType = "'b'"
    elif re.match(r"\d.\d.\d-a", branchVer) is not None:  # Alpha
        BuildType = "'a'"
    elif Branch == "develop":  # Devel
        BuildType = "'d'"
    else:
        BuildType = "'s'"  # Snapshot

    if Dirty:  # Dirty
        BuildType = "'*'"
    return BuildType


def print_human():  # Prints human-readable output
    print(f"Building version {str(FullVer)}"
          f" from repo {RepoName}"
          f" on branch {Branch}"
          f" at commit {str(Commit)[:8]}"
          f" on host {Hostname}"
          f" at time {str(Now)}")
    print(f"Build number: {BuildNumber:d}")
    print(f"Your Host key: 0x{HostKey}")
    print(f"Combined build ID: {HostKey}.{BuildNumber:d}\n")


def writer(fn):  # Intelligently detects asks to write to STDOUT and does so with `with`
    @contextlib.contextmanager
    def stdout():
        yield sys.stdout
    return stdout() if fn == "STDOUT" else open(fn, 'w', encoding="utf-8")


def write_build_info_file(file=None):  # Writes a build info file to the provided location
    file = BuildInfoFileLocation if file is None else file
    print(f"Writing Build Info file to {file}")
    with writer(file) as f:
        f.write(f"#define BUILD_NUMBER {BuildNumber:d}\n")
        f.write(f"#define BUILD_TYPE {BuildType}\n")
        f.write(f"#define BUILD_HOSTNAME \"{Hostname}\"\n")
        f.write(f"#define BUILD_HOST_KEY 0x{HostKey}\n")
        f.write(f"#define BUILD_DATE_YEAR {Now.year}\n")
        f.write(f"#define BUILD_DATE_MONTH {Now.month}\n")
        f.write(f"#define BUILD_DATE_DAY {Now.day}\n")
        f.write(f"#define BUILD_DATE \"{Now.date()}\"\n")
        f.write(f"#define BUILD_TIME_HOUR {Now.hour}\n")
        f.write(f"#define BUILD_TIME_MINUTE {Now.minute}\n")
        f.write(f"#define BUILD_TIME_SECOND {Now.second}\n")
        f.write(f"#define BUILD_TIME_MILLISECOND {int(Now.microsecond / 10000):d}\n")
        f.write(f"#define BUILD_TIME \"{BuildTime}\"\n")
    if file != "STDOUT":
        with open(file, "r", encoding="utf-8") as f:
            print(f.read())


def write_repo_info_file(file=None):  # Writes a repository info file to the provided location
    file = RepositoryInfoFileLocation if file is None else file
    print(f"Writing Repository Info file to {file}")
    with writer(file) as f:
        f.write(f"#define REPO_NAME \"{RepoName}\"\n")
        f.write(f"#define REPO_BRANCH \"{Branch}\"\n")
        f.write(f"#define REPO_DESCRIPTION \"{RepoDescription}\"\n")
        f.write(f"#define REPO_GIT_COMMIT 0x{Commit[:8]}\n")
        f.write(f"#define REPO_VERSION \"{str(FullVer)}\"\n")
        f.write(f"#define REPO_VERSION_NO_META \"{str(FullVer).split('+')[0]}\"\n")
        f.write(f"#define REPO_VERSION_MAJOR {str(FullVer.major)}\n")
        f.write(f"#define REPO_VERSION_MINOR {str(FullVer.minor)}\n")
        f.write(f"#define REPO_VERSION_REVIS {str(FullVer.patch)}\n")
        f.write(f"#define REPO_VERSION_PRERELEASE {PreReleaseVersion}\n")
        f.write(f"#define NSIS_VERSION_NUMBER "
                f"\"{FullVer.major}.{FullVer.minor}.{FullVer.patch}.{PreReleaseVersion}\"\n")
    if file != "STDOUT":
        with open(file, "r", encoding="utf-8") as f:
            print(f.read())


def write_zephyr_version_file(file=None):  # Writes a Zephyr VERSION file to the provided location
    file = ZephyrVersionFileLocation if file is None else file
    print(f"Writing Zephyr Version File to {file}")
    with writer(file) as f:
        f.write(f"VERSION_MAJOR = {str(FullVer.major)}\n")
        f.write(f"VERSION_MINOR = {str(FullVer.minor)}\n")
        f.write(f"PATCHLEVEL = {str(FullVer.patch)}\n")
        f.write(f"VERSION_TWEAK = {PreReleaseVersion}\n")
        f.write(f"EXTRAVERSION = {str(FullVer.prerelease)}\n")
    if file != "STDOUT":
        with open(file, "r", encoding="utf-8") as f:
            print(f.read())


def do_git():
    get_root_repo()
    pull_git_tags()
    get_repo_info()


def collect_system_data():
    get_build_number()
    get_hostname()
    get_host_key()


def generate_build_data():
    generate_version_data()
    compute_build_type()


def write_files():
    write_build_info_file()
    write_repo_info_file()
    write_zephyr_version_file()


def main():  # Main function for if used not as a library
    do_git()
    set_default_file_paths(sys.argv[1])
    collect_system_data()
    generate_build_data()
    write_files()


if __name__ == "__main__":  # If we are the running script file call main()
    main()
