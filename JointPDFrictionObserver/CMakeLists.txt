set(JPDFOCOMP_NAME JointPDFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE JPDFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${JPDFOCOMP_NAME} MODULE
			${JPDFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${JPDFOCOMP_NAME} ${LIBS})

set_target_properties(${JPDFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${JPDFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${JPDFOCOMP_NAME} DESTINATION ./PluginComponents)