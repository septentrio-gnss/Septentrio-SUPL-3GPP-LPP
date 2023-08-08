list(APPEND SEPTENTRIO_SOURCES 
    "include/sbf.hpp"
    "src/sbf.cpp"
)

add_library(septentrio ${SEPTENTRIO_SOURCES})
target_include_directories(septentrio PUBLIC "include/")
target_link_libraries(septentrio PRIVATE rtklib Threads::Threads)

if (USE_ASAN)
target_compile_options(septentrio PRIVATE -fsanitize=address,undefined,leak)
target_link_libraries(septentrio PRIVATE -fsanitize=address,undefined,leak)
endif (USE_ASAN)
