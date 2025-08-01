set(TPDFOCOMP_NAME TaskPDFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE TPDFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${TPDFOCOMP_NAME} MODULE
			${TPDFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TPDFOCOMP_NAME} ${LIBS})

set_target_properties(${TPDFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${TPDFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${TPDFOCOMP_NAME} DESTINATION ./PluginComponents)