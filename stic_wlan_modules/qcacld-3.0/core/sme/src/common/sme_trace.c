/*
 * Copyright (c) 2013-2018 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * DOC: smeTrace.c
 *  Implementation for trace related APIs
 *
 * Author Kiran Kumar Reddy CH L V
 */
#include "ani_global.h"          /* for tpAniSirGlobal */
#include "mac_trace.h"
#include "sme_trace.h"
#include "sme_internal.h"
#ifndef SME_TRACE_RECORD
void sme_trace_init(tpAniSirGlobal pMac)
{

}
#endif
#ifdef SME_TRACE_RECORD

static void sme_trace_dump(void *mac_ctx, tp_qdf_trace_record record,
			   uint16_t rec_index)
{
	switch (record->code) {
	case TRACE_CODE_SME_COMMAND:
		sme_debug("%04d %012llu %s S%d %-14s %-30s(0x%x)",
			rec_index, record->qtime, record->time, record->session,
			"SME COMMAND:",
			sme_trace_get_command_string(record->data),
			record->data);
		break;
	case TRACE_CODE_SME_TX_WMA_MSG:
		sme_debug("%04d %012llu %s S%d %-14s %-30s(0x%x)",
			rec_index, record->qtime, record->time, record->session,
			"TX WMA Msg:",
			mac_trace_get_wma_msg_string((uint16_t)record->data),
			record->data);
		break;
	case TRACE_CODE_SME_RX_WMA_MSG:
		sme_debug("%04d %012llu %s S%d %-14s %-30s(0x%x)",
			rec_index, record->qtime, record->time, record->session,
			"RX WMA Msg:",
			mac_trace_get_sme_msg_string((uint16_t)record->data),
			record->data);
		break;
	default:
		sme_debug("%04d %012llu %s S%d %-14s %-30s(0x%x)",
			rec_index, record->qtime, record->time, record->session,
			"RX HDD MSG:",
			sme_trace_get_rx_msg_string(record->code),
			record->data);
		break;
	}
}

void sme_trace_init(tpAniSirGlobal pMac)
{
	qdf_trace_register(QDF_MODULE_ID_SME, &sme_trace_dump);
}
#endif
