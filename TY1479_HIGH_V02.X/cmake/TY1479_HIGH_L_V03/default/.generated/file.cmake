# The following variables contains the files used by the different stages of the build process.
set(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemble)
set_source_files_properties(${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemblePreprocess)
set_source_files_properties(${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemblePreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_assemblePreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/device_config.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/eccp3.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/epwm1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/interrupt_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/mcc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/pin_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/tmr0.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/tmr1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/tmr2.c")
set_source_files_properties(${TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(TY1479_HIGH_L_V03_default_default_XC8_FILE_TYPE_link)
set(TY1479_HIGH_L_V03_default_image_name "default.elf")
set(TY1479_HIGH_L_V03_default_image_base_name "default")

# The output directory of the final image.
set(TY1479_HIGH_L_V03_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/TY1479_HIGH_L_V03")

# The full path to the final image.
set(TY1479_HIGH_L_V03_default_full_path_to_image ${TY1479_HIGH_L_V03_default_output_dir}/${TY1479_HIGH_L_V03_default_image_name})
