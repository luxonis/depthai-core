#include "GstPipelineInterface.hpp"

void GstPipelineInterface::run(bool run_in_thread) {
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "kmbcam");

    if(run_in_thread)
        g_thread_new("GstreamerLoop", (GThreadFunc)g_main_loop_run, main_loop);
    else
        g_main_loop_run(main_loop);
}
void GstPipelineInterface::stop() {
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_quit(main_loop);
}
