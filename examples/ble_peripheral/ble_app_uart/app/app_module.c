#include "common.h"
#include "app.h"

int app_module_init(void)
{
    NRF_LOG_INFO("app_module_init, start");
    // app_storage_init();
    app_bind_init();
    NRF_LOG_INFO("app_module_init, done");
    
    return 0;
}
