# cmake files support debug production
include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(TY1479_HIGH_L_V03_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemble)
add_library(TY1479_HIGH_L_V03_default_default_XC8_assemble OBJECT ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemble})
    TY1479_HIGH_L_V03_default_default_XC8_assemble_rule(TY1479_HIGH_L_V03_default_default_XC8_assemble)
    list(APPEND TY1479_HIGH_L_V03_default_library_list "$<TARGET_OBJECTS:TY1479_HIGH_L_V03_default_default_XC8_assemble>")
endif()

# Handle files with suffix S, for group default-XC8
if(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(TY1479_HIGH_L_V03_default_default_XC8_assemblePreprocess OBJECT ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemblePreprocess})
    TY1479_HIGH_L_V03_default_default_XC8_assemblePreprocess_rule(TY1479_HIGH_L_V03_default_default_XC8_assemblePreprocess)
    list(APPEND TY1479_HIGH_L_V03_default_library_list "$<TARGET_OBJECTS:TY1479_HIGH_L_V03_default_default_XC8_assemblePreprocess>")
endif()

# Handle files with suffix [cC], for group default-XC8
if(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_compile)
add_library(TY1479_HIGH_L_V03_default_default_XC8_compile OBJECT ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_compile})
    TY1479_HIGH_L_V03_default_default_XC8_compile_rule(TY1479_HIGH_L_V03_default_default_XC8_compile)
    list(APPEND TY1479_HIGH_L_V03_default_library_list "$<TARGET_OBJECTS:TY1479_HIGH_L_V03_default_default_XC8_compile>")
endif()

add_executable(TY1479_HIGH_L_V03_default_image_OBhppBsu ${TY1479_HIGH_L_V03_default_library_list})

set_target_properties(TY1479_HIGH_L_V03_default_image_OBhppBsu PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TY1479_HIGH_L_V03_default_output_dir})
set_target_properties(TY1479_HIGH_L_V03_default_image_OBhppBsu PROPERTIES OUTPUT_NAME "default")
set_target_properties(TY1479_HIGH_L_V03_default_image_OBhppBsu PROPERTIES SUFFIX ".elf")

target_link_libraries(TY1479_HIGH_L_V03_default_image_OBhppBsu PRIVATE ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_link})


# Add the link options from the rule file.
TY1479_HIGH_L_V03_default_link_rule(TY1479_HIGH_L_V03_default_image_OBhppBsu)



