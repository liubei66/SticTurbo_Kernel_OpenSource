/*
 * BFQ: I/O context handling.
 *
 * Based on ideas and code from CFQ:
 * Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 *
 * Copyright (C) 2008 Fabio Checconi <fabio@gandalf.sssup.it>
 *		      Paolo Valente <paolo.valente@unimore.it>
 *
 * Copyright (C) 2010 Paolo Valente <paolo.valente@unimore.it>
 */

static struct bfq_io_cq *icq_to_bic(struct io_cq *icq)
{
	return container_of(icq, struct bfq_io_cq, icq);
}

static struct bfq_io_cq *bfq_bic_lookup(struct bfq_data *bfqd,
					struct io_context *ioc)
{
	if (ioc)
		return icq_to_bic(ioc_lookup_icq(ioc, bfqd->queue));
	return NULL;
}
