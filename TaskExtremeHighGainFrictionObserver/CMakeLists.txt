set(THGFOCOMP_NAME TaskExtremeHighGainFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE THGFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${THGFOCOMP_NAME} MODULE
			${THGFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${THGFOCOMP_NAME} ${LIBS})

set_target_properties(${THGFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${THGFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${THGFOCOMP_NAME} DESTINATION ./PluginComponents)