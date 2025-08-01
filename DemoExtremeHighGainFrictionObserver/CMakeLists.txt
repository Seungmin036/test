set(DHGFOCOMP_NAME DemoExtremeHighGainFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE DHGFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${DHGFOCOMP_NAME} MODULE
			${DHGFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${DHGFOCOMP_NAME} ${LIBS})

set_target_properties(${DHGFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${DHGFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${DHGFOCOMP_NAME} DESTINATION ./PluginComponents)