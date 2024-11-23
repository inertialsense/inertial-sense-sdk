#ifndef SDK_VERSION_H
#define SDK_VERSION_H

// Note the two possible file name string formats.
#if (__has_include("version/repositoryInfo.h") && \
      __has_include("version/buildInfo.h"))
    #include "version/repositoryInfo.h"
    #include "version/buildInfo.h"
#elif (__has_include("../../cpp/EvalTool/version/repositoryInfo.h") && \
      __has_include("../../cpp/EvalTool/version/buildInfo.h"))
    #include "../../cpp/EvalTool/version/repositoryInfo.h"
    #include "../../cpp/EvalTool/version/buildInfo.h"
#else
    #define REPO_VERSION_MAJOR      2
    #define REPO_VERSION_MINOR      2
    #define REPO_VERSION_REVIS      0
    #define REPO_VERSION_PRERELEASE 0

    #define REPO_NAME               "is-sdk"
    #define REPO_BRANCH             "<no-git-info>"
    #define REPO_DESCRIPTION        "<no-git-info>"
    #define REPO_GIT_COMMIT         0xdeadbeaf
    #define REPO_VERSION            "<no-git-info>"
    #define REPO_VERSION_NO_META    "<no-git-info>"
    #define REPO_VERSION_MAJOR      2
    #define REPO_VERSION_MINOR      2
    #define REPO_VERSION_REVIS      0
    #define REPO_VERSION_PRERELEASE 0

    #define QU(x) #x
    #define QUH(x) QU(x)
    #define VERSION_STRING
    #define NSIS_VERSION_NUMBER QUH(REPO_VERSION_MAJOR) "." QUH(REPO_VERSION_MINOR) "." QUH(REPO_VERSION_REVIS) "." QUH(REPO_VERSION_PRERELEASE)
#endif

#define IS_SDK_VERSION_MAJOR  REPO_VERSION_MAJOR
#define IS_SDK_VERSION_MINOR  REPO_VERSION_MINOR
#define IS_SDK_VERSION_PATCH  REPO_VERSION_REVIS
#define IS_SDK_VERSION_REVIS  REPO_VERSION_PRERELEASE

#endif // SDK_VERSION_H
