#include <iostream>
#include <stdexcept>
#if 0
extern "C"
{
    void __btd_log_init(const char* debug, int detach) {}
    void __btd_log_cleanup() {}
    void info(const char* format, ...) {}
    void btd_debug(uint16_t index, const char* format, ...) {}
    void btd_info(uint16_t index, const char* format, ...) {}
    void btd_warn(uint16_t index, const char* format, ...) {}
    void btd_error(uint16_t index, const char* format, ...) {}
}
#endif
void UndefinedFuncThrow(const char* sym)
try
{
    std::cerr << "Called unimplemented sym : " << sym << std::endl;
    throw std::logic_error(sym);
} catch (...)
{}
void UndefinedFunc(const char* sym)
{}
#define DUMMY_DEFINE(sym)         \
    extern "C" int sym()          \
    {                             \
        UndefinedFuncThrow(#sym); \
        return 1;                 \
    }
#define VERIFIED_DUMMY_DEFINE(sym) \
    extern "C" int sym()           \
    {                              \
        UndefinedFunc(#sym);       \
        return 1;                  \
    }
VERIFIED_DUMMY_DEFINE(g_dbus_register_interface)
VERIFIED_DUMMY_DEFINE(g_dbus_unregister_interface)
VERIFIED_DUMMY_DEFINE(g_dbus_emit_property_changed)
VERIFIED_DUMMY_DEFINE(g_dbus_emit_property_changed_full)
extern "C" int g_dbus_get_flags()
{
    return 0;
}

DUMMY_DEFINE(dbus_connection_get_is_connected)
DUMMY_DEFINE(dbus_connection_unref)
DUMMY_DEFINE(dbus_error_free)
DUMMY_DEFINE(dbus_error_has_name)
DUMMY_DEFINE(dbus_error_init)
DUMMY_DEFINE(dbus_error_is_set)
DUMMY_DEFINE(dbus_message_append_args)
DUMMY_DEFINE(dbus_message_get_args)
DUMMY_DEFINE(dbus_message_get_member)
DUMMY_DEFINE(dbus_message_get_sender)
DUMMY_DEFINE(dbus_message_has_member)
DUMMY_DEFINE(dbus_message_is_method_call)
DUMMY_DEFINE(dbus_message_iter_append_basic)
DUMMY_DEFINE(dbus_message_iter_append_fixed_array)
DUMMY_DEFINE(dbus_message_iter_close_container)
DUMMY_DEFINE(dbus_message_iter_get_arg_type)
DUMMY_DEFINE(dbus_message_iter_get_basic)
DUMMY_DEFINE(dbus_message_iter_get_element_type)
DUMMY_DEFINE(dbus_message_iter_get_fixed_array)
DUMMY_DEFINE(dbus_message_iter_has_next)
DUMMY_DEFINE(dbus_message_iter_init)
DUMMY_DEFINE(dbus_message_iter_init_append)
DUMMY_DEFINE(dbus_message_iter_next)
DUMMY_DEFINE(dbus_message_iter_open_container)
DUMMY_DEFINE(dbus_message_iter_recurse)
DUMMY_DEFINE(dbus_message_new_error)
DUMMY_DEFINE(dbus_message_new_method_call)
DUMMY_DEFINE(dbus_message_new_method_return)
DUMMY_DEFINE(dbus_message_ref)
DUMMY_DEFINE(dbus_message_unref)
DUMMY_DEFINE(dbus_pending_call_cancel)
DUMMY_DEFINE(dbus_pending_call_set_notify)
DUMMY_DEFINE(dbus_pending_call_steal_reply)
DUMMY_DEFINE(dbus_pending_call_unref)
DUMMY_DEFINE(dbus_set_error_const)
DUMMY_DEFINE(dbus_set_error_from_message)
DUMMY_DEFINE(g_dbus_add_disconnect_watch)
DUMMY_DEFINE(g_dbus_add_service_watch)
DUMMY_DEFINE(g_dbus_attach_object_manager)
DUMMY_DEFINE(g_dbus_client_new)
DUMMY_DEFINE(g_dbus_client_new_full)
DUMMY_DEFINE(g_dbus_client_set_disconnect_watch)
DUMMY_DEFINE(g_dbus_client_set_proxy_handlers)
DUMMY_DEFINE(g_dbus_client_set_ready_watch)
DUMMY_DEFINE(g_dbus_client_unref)
DUMMY_DEFINE(g_dbus_create_error)
DUMMY_DEFINE(g_dbus_create_reply)
DUMMY_DEFINE(g_dbus_detach_object_manager)
DUMMY_DEFINE(g_dbus_dict_append_array)
DUMMY_DEFINE(g_dbus_dict_append_basic_array)
DUMMY_DEFINE(g_dbus_dict_append_entry)
DUMMY_DEFINE(g_dbus_pending_property_error)
DUMMY_DEFINE(g_dbus_pending_property_success)
DUMMY_DEFINE(g_dbus_proxy_get_interface)
DUMMY_DEFINE(g_dbus_proxy_get_path)
DUMMY_DEFINE(g_dbus_proxy_get_property)
DUMMY_DEFINE(g_dbus_proxy_method_call)
DUMMY_DEFINE(g_dbus_proxy_new)
DUMMY_DEFINE(g_dbus_proxy_ref)
DUMMY_DEFINE(g_dbus_proxy_refresh_property)
DUMMY_DEFINE(g_dbus_proxy_set_property_basic)
DUMMY_DEFINE(g_dbus_proxy_set_property_watch)
DUMMY_DEFINE(g_dbus_proxy_unref)
DUMMY_DEFINE(g_dbus_remove_watch)
DUMMY_DEFINE(g_dbus_send_error)
DUMMY_DEFINE(g_dbus_send_message)
DUMMY_DEFINE(g_dbus_send_message_with_reply)
DUMMY_DEFINE(g_dbus_send_reply)
DUMMY_DEFINE(g_dbus_set_disconnect_function)
DUMMY_DEFINE(g_dbus_set_flags)
DUMMY_DEFINE(g_dbus_setup_bus)
DUMMY_DEFINE(rl_printf)
