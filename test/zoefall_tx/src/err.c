/*
    logs
*/
#define TAG "ZF_TX_ERR"
#define FILE_ID 0x21
#include "zoecare/logs/logs.h"

// -------------------------------------------------------------------
#pragma region INCLUDES
// -------------------------------------------------------------------

#include "zoecare/err/err.h"
#include "zoecare/report/report.h"

// -------------------------------------------------------------------
#pragma region CALLBACKS
// -------------------------------------------------------------------

static zc_err_backtrace_t *get_bt_from_tls(void)
{
    return pvTaskGetThreadLocalStoragePointer(NULL, 1);
}

static void catch_cb(zc_err_backtrace_t *bt)
{
    if (bt == NULL) return;
    ZC_TRY(zc_report_bt, bt);
}

// -------------------------------------------------------------------
#pragma region INIT
// -------------------------------------------------------------------

void zc_zf_tx_err_init(void)
{
    zc_err_get_bt = get_bt_from_tls;
    zc_err_catch_cb = catch_cb;
}
