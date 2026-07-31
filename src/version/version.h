/**
 * @file version.h
 * @brief Includes the generated version/build-info headers (repositoryInfo.h and buildInfo.h,
 *        which define the IS_SDK_REPO_ constants for the actual repo/build) when they can be
 *        found, or falls back to hard-coded REPO_ placeholder constants otherwise; also defines
 *        the IS_SDK_VERSION_MAJOR/MINOR/PATCH/REVIS aliases (pulled from the generated
 *        IS_SDK_REPO_VERSION_ constants) that the rest of the SDK consumes.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

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
    #define REPO_VERSION_MAJOR      2       //!< fallback major version, used when repositoryInfo.h/buildInfo.h were not generated
    #define REPO_VERSION_MINOR      7       //!< fallback minor version, used when repositoryInfo.h/buildInfo.h were not generated
    #define REPO_VERSION_REVIS      0       //!< fallback revision/patch version, used when repositoryInfo.h/buildInfo.h were not generated
    #define REPO_VERSION_PRERELEASE 0       //!< fallback prerelease/build number, used when repositoryInfo.h/buildInfo.h were not generated

    #define REPO_NAME               "is-sdk"          //!< fallback repository name
    #define REPO_BRANCH             "<no-git-info>"   //!< fallback git branch name
    #define REPO_DESCRIPTION        "<no-git-info>"   //!< fallback git describe output
    #define REPO_GIT_COMMIT         0xdeadbeaf         //!< fallback git commit hash placeholder
    #define REPO_VERSION            "<no-git-info>"   //!< fallback full version string (with metadata)
    #define REPO_VERSION_NO_META    "<no-git-info>"   //!< fallback version string without build metadata
    #define REPO_VERSION_MAJOR      2       //!< fallback major version (duplicate definition, kept for compatibility)
    #define REPO_VERSION_MINOR      7       //!< fallback minor version (duplicate definition, kept for compatibility)
    #define REPO_VERSION_REVIS      0       //!< fallback revision/patch version (duplicate definition, kept for compatibility)
    #define REPO_VERSION_PRERELEASE 0       //!< fallback prerelease/build number (duplicate definition, kept for compatibility)

    #define QU(x) #x                        //!< stringizes x without macro-expanding it first
    #define QUH(x) QU(x)                    //!< expands x, then stringizes the result (use this for macro arguments)
    #define VERSION_STRING                  //!< placeholder marker; intentionally empty in the fallback branch
    #define NSIS_VERSION_NUMBER QUH(REPO_VERSION_MAJOR) "." QUH(REPO_VERSION_MINOR) "." QUH(REPO_VERSION_REVIS) "." QUH(REPO_VERSION_PRERELEASE)   //!< dotted "major.minor.revis.prerelease" string consumed by the NSIS installer script
#endif

#define IS_SDK_VERSION_MAJOR  IS_SDK_REPO_VERSION_MAJOR        //!< SDK major version, aliasing IS_SDK_REPO_VERSION_MAJOR from the generated repositoryInfo.h
#define IS_SDK_VERSION_MINOR  IS_SDK_REPO_VERSION_MINOR        //!< SDK minor version, aliasing IS_SDK_REPO_VERSION_MINOR from the generated repositoryInfo.h
#define IS_SDK_VERSION_PATCH  IS_SDK_REPO_VERSION_REVIS        //!< SDK patch version, aliasing IS_SDK_REPO_VERSION_REVIS from the generated repositoryInfo.h
#define IS_SDK_VERSION_REVIS  IS_SDK_REPO_VERSION_PRERELEASE   //!< SDK prerelease/build number, aliasing IS_SDK_REPO_VERSION_PRERELEASE from the generated repositoryInfo.h

#endif // SDK_VERSION_H
