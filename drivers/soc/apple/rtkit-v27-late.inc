/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Included by rtkit.c. Existing common RTKit/mailbox function bodies unchanged. */
#include <linux/module.h>
struct apple_rtkit_v27_scope {
	struct apple_rtkit *rtk;
	struct module *provider_module;
	DECLARE_KFIFO(deferred, struct apple_mbox_msg, 64);
	bool held, stopped, failed;
	int error;
	/* EXPERIMENTAL: run once between the AP acknowledgment and the IOP request. */
	void (*after_ap)(void *data);
	void *after_ap_data;
};

void apple_rtkit_v27_set_after_ap(struct apple_rtkit_v27_scope *s, void (*fn)(void *), void *data)
{
	if (s) {
		s->after_ap = fn;
		s->after_ap_data = data;
	}
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_set_after_ap);

bool apple_rtkit_v27_live(struct apple_rtkit *rtk)
{
	return !rtk->crashed && !rtk->shutting_down &&
		rtk->ap_power_state == APPLE_RTKIT_PWR_STATE_ON &&
		rtk->iop_power_state == APPLE_RTKIT_PWR_STATE_ON;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_live);

int apple_rtkit_v27_disarm(struct apple_rtkit_v27_scope *s, bool stopped)
{
	struct apple_mbox_msg msg;
	struct apple_rtkit *rtk;

	if (!s || !s->held)
		return -EINVAL;
	rtk = s->rtk;
	if (s->failed || (stopped && !s->stopped))
		return -EUCLEAN; /* retain reservation/storage/RX on uncertain stop */
	/* Caller has verified restored provider power before stopped=true. */
	if (stopped)
		apple_mbox_set_recv_enabled(rtk->mbox, true);
	s->held = false;
	mutex_unlock(&rtk->mbox->lifecycle_lock);
	/* Process context, outside provider lifecycle lock. Do not discard an
	 * accepted packet just because precommit admission had to refuse.
	 */
	while (kfifo_get(&s->deferred, &msg))
		apple_rtkit_rx_dispatch(rtk, msg.msg1, msg.msg0, false);
	apple_mbox_release_rx(rtk->mbox);
	module_put(s->provider_module);
	put_device(rtk->mbox->dev);
	kfree(s);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_disarm);

int apple_rtkit_v27_arm(struct apple_rtkit *rtk, struct apple_rtkit_v27_scope **result)
{
	struct apple_rtkit_v27_scope *s;
	struct apple_mbox *b = rtk->mbox;
	struct apple_mbox_msg msg;
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) ||
	    !apple_rtkit_v27_rx_drained(rtk) || rtk->strict_idle_poisoned || rtk->crashed ||
	    rtk->shutting_down || rtk->ap_power_state != APPLE_RTKIT_PWR_STATE_ON ||
	    rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_ON || b->irq_recv_not_empty < 0 ||
	    !apple_mbox_poll_one_supported(b))
		return -EBUSY;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	if (!mutex_trylock(&b->lifecycle_lock)) {
		kfree(s);
		return -EBUSY;
	}
	if (b->removing || !b->active || b->cookie != rtk || b->rx != apple_rtkit_rx ||
	    !b->dev->driver || !try_module_get(b->dev->driver->owner)) {
		mutex_unlock(&b->lifecycle_lock);
		kfree(s);
		return -ENODEV;
	}
	s->provider_module = b->dev->driver->owner;
	get_device(b->dev);
	s->rtk = rtk;
	s->held = true;
	INIT_KFIFO(s->deferred);
	apple_mbox_hold_rx(b); /* process-context IRQ synchronization, never final commit */
	if (!apple_rtkit_v27_rx_drained(rtk)) {
		ret = -EBUSY;
		goto undo;
	}
	ret = apple_mbox_recv_nowait(b, &msg);
	if (ret > 0) {
		kfifo_put(&s->deferred, msg);
		ret = -EAGAIN;
	}
	if (ret)
		goto undo;
	*result = s;
	return 0;
undo:
	apple_rtkit_v27_disarm(s, false);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_arm);

int apple_rtkit_v27_check(struct apple_rtkit_v27_scope *s)
{
	struct apple_mbox_msg msg;
	int ret;

	if (!s || !s->held || s->failed || s->stopped ||
	    s->rtk->mbox->removing || !s->rtk->mbox->active ||
	    s->rtk->crashed || !apple_rtkit_v27_rx_drained(s->rtk) ||
	    s->rtk->ap_power_state != APPLE_RTKIT_PWR_STATE_ON ||
	    s->rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_ON)
		return -EBUSY;
	if (kfifo_is_full(&s->deferred))
		return -ENOBUFS;
	ret = apple_mbox_recv_nowait(s->rtk->mbox, &msg);
	if (ret > 0) {
		kfifo_put(&s->deferred, msg);
		return -EAGAIN;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_check);

/*
 * EXPERIMENTAL (K116s deep sleep): the idle's budget in 10 us ticks. 10000 (100 ms) is what the
 * J714s on firmware 27.0 needs; firmware 27.2 has not been seen to finish in it.
 */
static unsigned int v27_idle_ticks = 100000;
module_param(v27_idle_ticks, uint, 0644);
/*
 * EXPERIMENTAL: the power states the atomic idle asks for. 0x201 (idle, state retained) is
 * macOS's on firmware 27.0; firmware 27.2's display coprocessor does not acknowledge it for
 * the IOP. 0x10 is quiesced, 0x1 sleep. The idle notice is only awaited for 0x201.
 */
static unsigned int v27_idle_ap_state = APPLE_RTKIT_PWR_STATE_IDLE;
module_param(v27_idle_ap_state, uint, 0644);
static unsigned int v27_idle_iop_state = APPLE_RTKIT_PWR_STATE_IDLE;
module_param(v27_idle_iop_state, uint, 0644);

/*
 * EXPERIMENTAL (K116s deep sleep): a second port's AP idle with interrupts off, sent and
 * awaited by polling its mailbox for at most @us. The port's tracked state is not updated.
 */
int apple_rtkit_v27_port_ap_idle_atomic(struct apple_rtkit *rtk, unsigned int us)
{
	struct apple_mbox_msg msg = {
		.msg0 = FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE) |
			FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, APPLE_RTKIT_PWR_STATE_IDLE),
		.msg1 = APPLE_RTKIT_EP_MGMT,
	};
	unsigned int i, n = us / 10;
	int ret = -ETIMEDOUT;

	for (i = 0; i < n; i++) {
		ret = apple_mbox_send_nowait(rtk->mbox, msg);
		if (ret != -EAGAIN && ret != -EBUSY)
			break;
		udelay(10);
	}
	if (ret)
		return ret;
	for (; i < n; i++) {
		struct apple_mbox_msg in;

		ret = apple_mbox_recv_nowait(rtk->mbox, &in);
		if (ret < 0 && ret != -EBUSY)
			return ret;
		if (ret > 0 && in.msg1 == APPLE_RTKIT_EP_MGMT &&
		    FIELD_GET(APPLE_RTKIT_MGMT_TYPE, in.msg0) == APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_port_ap_idle_atomic);

int apple_rtkit_v27_idle_atomic(struct apple_rtkit_v27_scope *s)
{
	struct apple_rtkit *rtk;
	struct apple_mbox_msg msg, echo = {};
	bool ap_sent = false, ap_ack = false, iop_sent = false, iop_ack = false;
	bool done = false, echo_pending = false;
	unsigned int tick, rx = 0, ap_tick = 0, iop_tick = 0, done_tick = 0;
	struct apple_mbox_msg last = {};
	int ret = -ETIMEDOUT;

	if (!s || !s->held || s->failed || !irqs_disabled())
		return -EINVAL;
	rtk = s->rtk;
	/* One shared 100 ms budget. No jiffies, allocation, worker, sleeping
	 * locks, normal send, or unbounded RX drain in saved context.
	 */
	for (tick = 0; tick < v27_idle_ticks; tick++) {
		if (rtk->crashed || rtk->mbox->removing) {
			ret = -EIO;
			goto poison;
		}
		if (echo_pending) {
			ret = apple_mbox_send_nowait(rtk->mbox, echo);
			if (!ret)
				echo_pending = false;
		} else if (!ap_sent || (ap_ack && !iop_sent)) {
			bool ap = !ap_sent;

			msg.msg1 = APPLE_RTKIT_EP_MGMT;
			msg.msg0 = FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, ap ?
				APPLE_RTKIT_MGMT_SET_AP_PWR_STATE : APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE) |
				FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE,
					   ap ? v27_idle_ap_state : v27_idle_iop_state);
			dma_wmb();
			ret = apple_mbox_send_nowait(rtk->mbox, msg);
			if (!ret) {
				if (ap)
					ap_sent = true;
				else
					iop_sent = true;
			}
		} else {
			ret = 0;
		}
		if (ret && ret != -EAGAIN && ret != -EBUSY)
			goto poison;
		/* Reserve storage before consuming any potentially foreign packet. */
		if (kfifo_is_full(&s->deferred)) {
			ret = -ENOBUFS;
			goto poison;
		}
		if (!echo_pending) {
			ret = apple_mbox_recv_nowait(rtk->mbox, &msg);
			if (ret < 0 && ret != -EBUSY)
				goto poison;
			if (ret > 0) {
				u8 type = FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg.msg0);
				u64 arg = msg.msg0 & GENMASK_ULL(51, 0);

				dma_rmb();
				rx++;
				last = msg;
				if (msg.msg1 == APPLE_RTKIT_EP_MGMT && ap_sent && !ap_ack &&
				    type == APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK &&
				    arg == v27_idle_ap_state) {
					ap_ack = true;
					ap_tick = tick;
					if (s->after_ap)
						s->after_ap(s->after_ap_data);
				} else if (msg.msg1 == APPLE_RTKIT_EP_MGMT && iop_sent && !iop_ack &&
					   type == APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE_ACK &&
					   arg == v27_idle_iop_state) {
					iop_ack = true;
					iop_tick = tick;
				} else if (msg.msg1 == APPLE_RTKIT_EP_MGMT && iop_sent && !done &&
					   type == APPLE_RTKIT_MGMT_IDLE_NOTIFY && !arg) {
					done = true;
					done_tick = tick;
				} else if (apple_rtkit_oslog_owed_msg(rtk, msg.msg1, msg.msg0)) {
					echo = msg;
					echo_pending = true;
				} else {
					kfifo_put(&s->deferred, msg);
					ret = -EPROTO;
					goto poison;
				}
			}
		}
		if (ap_ack && iop_ack && !echo_pending &&
		    (done || v27_idle_iop_state != APPLE_RTKIT_PWR_STATE_IDLE)) {
			dev_info(rtk->dev, "RTKit: atomic idle: AP ack tick %u, IOP ack tick %u, idle notice tick %u, %u received\n",
				 ap_tick, iop_tick, done_tick, rx);
			rtk->ap_power_state = rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_IDLE;
			s->stopped = true;
			apple_mbox_set_recv_enabled(rtk->mbox, false);
			return 0;
		}
		udelay(10);
	}
	ret = -ETIMEDOUT;
poison:
	/* EXPERIMENTAL: what the firmware answered. The caller panics; this is the only record. */
	dev_emerg(rtk->dev,
		  "RTKit: atomic idle failed %d at tick %u: AP sent %d ack %d (tick %u), IOP sent %d ack %d (tick %u), idle notice %d (tick %u), echo pending %d, %u received, last ep %#llx msg %#llx\n",
		  ret, tick, ap_sent, ap_ack, ap_tick, iop_sent, iop_ack, iop_tick, done, done_tick,
		  echo_pending, rx, (u64)last.msg1, (u64)last.msg0);
	s->failed = true;
	s->error = ret;
	rtk->strict_idle_poisoned = true;
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_idle_atomic);
