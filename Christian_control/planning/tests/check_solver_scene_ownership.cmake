if (NOT DEFINED SOURCE)
    message(FATAL_ERROR "SOURCE is required")
endif ()

file(READ "${SOURCE}" solver_source)

function(require_one_scene_sdf section label)
    string(REGEX MATCHALL "MakeMountSdf\\(" all_sdf_calls "${section}")
    list(LENGTH all_sdf_calls all_sdf_count)
    string(REGEX MATCHALL "MakeMountSdf\\([^;]*config\\.scene\\)"
           scene_sdf_calls "${section}")
    list(LENGTH scene_sdf_calls scene_sdf_count)
    if (NOT all_sdf_count EQUAL 1 OR NOT scene_sdf_count EQUAL 1)
        message(FATAL_ERROR
            "${label} must contain exactly one MakeMountSdf call and it must consume config.scene "
            "(all calls=${all_sdf_count}, scene calls=${scene_sdf_count})")
    endif ()
endfunction()

set(point_start_token "PlanOutcome SolveToPosition(")
set(path_section_token "// ---------------------------------------------------------------\n// Cartesian path following")
set(path_start_token "PathPlanOutcome SolveAlongPath(")

string(FIND "${solver_source}" "${point_start_token}" point_start)
string(FIND "${solver_source}" "${path_section_token}" point_end)
string(FIND "${solver_source}" "${path_start_token}" path_start)
if (point_start EQUAL -1 OR point_end EQUAL -1 OR path_start EQUAL -1 OR
    NOT point_start LESS point_end OR NOT point_end LESS path_start)
    message(FATAL_ERROR "could not isolate both PlanSolver routes")
endif ()

math(EXPR point_length "${point_end} - ${point_start}")
string(SUBSTRING "${solver_source}" ${point_start} ${point_length} point_section)
string(SUBSTRING "${solver_source}" ${path_start} -1 path_section)

require_one_scene_sdf("${point_section}" "SolveToPosition")
require_one_scene_sdf("${path_section}" "SolveAlongPath")
