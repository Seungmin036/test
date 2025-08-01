set(JHGFOCOMP_NAME JointExtremeHighGainFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE JHGFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${JHGFOCOMP_NAME} MODULE
			${JHGFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${JHGFOCOMP_NAME} ${LIBS})

set_target_properties(${JHGFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${JHGFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${JHGFOCOMP_NAME} DESTINATION ./PluginComponents)