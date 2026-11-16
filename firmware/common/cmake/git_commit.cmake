# Generates git_commit.hh defining ADSBEE_GIT_COMMIT, the short commit of the build with "-dirty" for uncommitted
# changes, or "" when unknown. Docker builds mount the sources without .git, so build.sh passes the commit as
# -DADSBEE_GIT_COMMIT. The commit is read at configure time, so a new commit shows up after the next reconfigure.
#
# Usage:
#   include(<this file>)
#   target_include_directories(<target> PRIVATE ${ADSBEE_GIT_COMMIT_INCLUDE_DIR})

if(NOT DEFINED ADSBEE_GIT_COMMIT)
    execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                    OUTPUT_VARIABLE ADSBEE_GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(ADSBEE_GIT_COMMIT)
        execute_process(COMMAND git status --porcelain --untracked-files=no WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                        OUTPUT_VARIABLE _adsbee_git_changes ERROR_QUIET)
        if(_adsbee_git_changes)
            string(APPEND ADSBEE_GIT_COMMIT "-dirty")
        endif()
    endif()
endif()
message(STATUS "Git commit: ${ADSBEE_GIT_COMMIT}")

set(ADSBEE_GIT_COMMIT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/generated")
file(CONFIGURE OUTPUT "${ADSBEE_GIT_COMMIT_INCLUDE_DIR}/git_commit.hh"
     CONTENT "#pragma once\n\n#define ADSBEE_GIT_COMMIT \"${ADSBEE_GIT_COMMIT}\"\n")
