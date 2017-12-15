
# install to /bin by default
install(TARGETS ${CPP_FILE} DESTINATION ../bin)

aux_source_directory(. tracking_SRCS)

ADD_LIBRARY(tracking ${tracking_SRCS})
