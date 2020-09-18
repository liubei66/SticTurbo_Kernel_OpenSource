#ifndef __CFG80211_RDEV_OPS
#define __CFG80211_RDEV_OPS

#include <linux/rtnetlink.h>
#include <net/cfg80211.h>
#include "core.h"
#include "trace.h"

static inline int rdev_suspend(struct cfg80211_registered_device *rdev,
			       struct cfg80211_wowlan *wowlan)
{
	int ret;
	ret = rdev->ops->suspend(&rdev->wiphy, wowlan);
	return ret;
}

static inline int rdev_resume(struct cfg80211_registered_device *rdev)
{
	int ret;
	ret = rdev->ops->resume(&rdev->wiphy);
	return ret;
}

static inline void rdev_set_wakeup(struct cfg80211_registered_device *rdev,
				   bool enabled)
{
	rdev->ops->set_wakeup(&rdev->wiphy, enabled);
}

static inline struct wireless_dev
*rdev_add_virtual_intf(struct cfg80211_registered_device *rdev, char *name,
		       unsigned char name_assign_type,
		       enum nl80211_iftype type, u32 *flags,
		       struct vif_params *params)
{
	struct wireless_dev *ret;
	ret = rdev->ops->add_virtual_intf(&rdev->wiphy, name, name_assign_type,
					  type, flags, params);
	return ret;
}

static inline int
rdev_del_virtual_intf(struct cfg80211_registered_device *rdev,
		      struct wireless_dev *wdev)
{
	int ret;
	ret = rdev->ops->del_virtual_intf(&rdev->wiphy, wdev);
	return ret;
}

static inline int
rdev_change_virtual_intf(struct cfg80211_registered_device *rdev,
			 struct net_device *dev, enum nl80211_iftype type,
			 u32 *flags, struct vif_params *params)
{
	int ret;
	ret = rdev->ops->change_virtual_intf(&rdev->wiphy, dev, type, flags,
					     params);
	return ret;
}

static inline int rdev_add_key(struct cfg80211_registered_device *rdev,
			       struct net_device *netdev, u8 key_index,
			       bool pairwise, const u8 *mac_addr,
			       struct key_params *params)
{
	int ret;
	ret = rdev->ops->add_key(&rdev->wiphy, netdev, key_index, pairwise,
				  mac_addr, params);
	return ret;
}

static inline int
rdev_get_key(struct cfg80211_registered_device *rdev, struct net_device *netdev,
	     u8 key_index, bool pairwise, const u8 *mac_addr, void *cookie,
	     void (*callback)(void *cookie, struct key_params*))
{
	int ret;
	ret = rdev->ops->get_key(&rdev->wiphy, netdev, key_index, pairwise,
				  mac_addr, cookie, callback);
	return ret;
}

static inline int rdev_del_key(struct cfg80211_registered_device *rdev,
			       struct net_device *netdev, u8 key_index,
			       bool pairwise, const u8 *mac_addr)
{
	int ret;
	ret = rdev->ops->del_key(&rdev->wiphy, netdev, key_index, pairwise,
				  mac_addr);
	return ret;
}

static inline int
rdev_set_default_key(struct cfg80211_registered_device *rdev,
		     struct net_device *netdev, u8 key_index, bool unicast,
		     bool multicast)
{
	int ret;
	ret = rdev->ops->set_default_key(&rdev->wiphy, netdev, key_index,
					  unicast, multicast);
	return ret;
}

static inline int
rdev_set_default_mgmt_key(struct cfg80211_registered_device *rdev,
			  struct net_device *netdev, u8 key_index)
{
	int ret;
	ret = rdev->ops->set_default_mgmt_key(&rdev->wiphy, netdev,
					       key_index);
	return ret;
}

static inline int rdev_start_ap(struct cfg80211_registered_device *rdev,
				struct net_device *dev,
				struct cfg80211_ap_settings *settings)
{
	int ret;
	ret = rdev->ops->start_ap(&rdev->wiphy, dev, settings);
	return ret;
}

static inline int rdev_change_beacon(struct cfg80211_registered_device *rdev,
				     struct net_device *dev,
				     struct cfg80211_beacon_data *info)
{
	int ret;
	ret = rdev->ops->change_beacon(&rdev->wiphy, dev, info);
	return ret;
}

static inline int rdev_stop_ap(struct cfg80211_registered_device *rdev,
			       struct net_device *dev)
{
	int ret;
	ret = rdev->ops->stop_ap(&rdev->wiphy, dev);
	return ret;
}

static inline int rdev_add_station(struct cfg80211_registered_device *rdev,
				   struct net_device *dev, u8 *mac,
				   struct station_parameters *params)
{
	int ret;
	ret = rdev->ops->add_station(&rdev->wiphy, dev, mac, params);
	return ret;
}

static inline int rdev_del_station(struct cfg80211_registered_device *rdev,
				   struct net_device *dev,
				   struct station_del_parameters *params)
{
	int ret;
	ret = rdev->ops->del_station(&rdev->wiphy, dev, params);
	return ret;
}

static inline int rdev_change_station(struct cfg80211_registered_device *rdev,
				      struct net_device *dev, u8 *mac,
				      struct station_parameters *params)
{
	int ret;
	ret = rdev->ops->change_station(&rdev->wiphy, dev, mac, params);
	return ret;
}

static inline int rdev_get_station(struct cfg80211_registered_device *rdev,
				   struct net_device *dev, const u8 *mac,
				   struct station_info *sinfo)
{
	int ret;
	ret = rdev->ops->get_station(&rdev->wiphy, dev, mac, sinfo);
	return ret;
}

static inline int rdev_dump_station(struct cfg80211_registered_device *rdev,
				    struct net_device *dev, int idx, u8 *mac,
				    struct station_info *sinfo)
{
	int ret;
	ret = rdev->ops->dump_station(&rdev->wiphy, dev, idx, mac, sinfo);
	return ret;
}

static inline int rdev_add_mpath(struct cfg80211_registered_device *rdev,
				 struct net_device *dev, u8 *dst, u8 *next_hop)
{
	int ret;
	ret = rdev->ops->add_mpath(&rdev->wiphy, dev, dst, next_hop);
	return ret;
}

static inline int rdev_del_mpath(struct cfg80211_registered_device *rdev,
				 struct net_device *dev, u8 *dst)
{
	int ret;
	ret = rdev->ops->del_mpath(&rdev->wiphy, dev, dst);
	return ret;
}

static inline int rdev_change_mpath(struct cfg80211_registered_device *rdev,
				    struct net_device *dev, u8 *dst,
				    u8 *next_hop)
{
	int ret;
	ret = rdev->ops->change_mpath(&rdev->wiphy, dev, dst, next_hop);
	return ret;
}

static inline int rdev_get_mpath(struct cfg80211_registered_device *rdev,
				 struct net_device *dev, u8 *dst, u8 *next_hop,
				 struct mpath_info *pinfo)
{
	int ret;
	ret = rdev->ops->get_mpath(&rdev->wiphy, dev, dst, next_hop, pinfo);
	return ret;

}

static inline int rdev_get_mpp(struct cfg80211_registered_device *rdev,
			       struct net_device *dev, u8 *dst, u8 *mpp,
			       struct mpath_info *pinfo)
{
	int ret;

	ret = rdev->ops->get_mpp(&rdev->wiphy, dev, dst, mpp, pinfo);
	return ret;
}

static inline int rdev_dump_mpath(struct cfg80211_registered_device *rdev,
				  struct net_device *dev, int idx, u8 *dst,
				  u8 *next_hop, struct mpath_info *pinfo)

{
	int ret;
	ret = rdev->ops->dump_mpath(&rdev->wiphy, dev, idx, dst, next_hop,
				    pinfo);
	return ret;
}

static inline int rdev_dump_mpp(struct cfg80211_registered_device *rdev,
				struct net_device *dev, int idx, u8 *dst,
				u8 *mpp, struct mpath_info *pinfo)

{
	int ret;

	ret = rdev->ops->dump_mpp(&rdev->wiphy, dev, idx, dst, mpp, pinfo);
	return ret;
}

static inline int
rdev_get_mesh_config(struct cfg80211_registered_device *rdev,
		     struct net_device *dev, struct mesh_config *conf)
{
	int ret;
	ret = rdev->ops->get_mesh_config(&rdev->wiphy, dev, conf);
	return ret;
}

static inline int
rdev_update_mesh_config(struct cfg80211_registered_device *rdev,
			struct net_device *dev, u32 mask,
			const struct mesh_config *nconf)
{
	int ret;
	ret = rdev->ops->update_mesh_config(&rdev->wiphy, dev, mask, nconf);
	return ret;
}

static inline int rdev_join_mesh(struct cfg80211_registered_device *rdev,
				 struct net_device *dev,
				 const struct mesh_config *conf,
				 const struct mesh_setup *setup)
{
	int ret;
	ret = rdev->ops->join_mesh(&rdev->wiphy, dev, conf, setup);
	return ret;
}


static inline int rdev_leave_mesh(struct cfg80211_registered_device *rdev,
				  struct net_device *dev)
{
	int ret;
	ret = rdev->ops->leave_mesh(&rdev->wiphy, dev);
	return ret;
}

static inline int rdev_join_ocb(struct cfg80211_registered_device *rdev,
				struct net_device *dev,
				struct ocb_setup *setup)
{
	int ret;
	ret = rdev->ops->join_ocb(&rdev->wiphy, dev, setup);
	return ret;
}

static inline int rdev_leave_ocb(struct cfg80211_registered_device *rdev,
				 struct net_device *dev)
{
	int ret;
	ret = rdev->ops->leave_ocb(&rdev->wiphy, dev);
	return ret;
}

static inline int rdev_change_bss(struct cfg80211_registered_device *rdev,
				  struct net_device *dev,
				  struct bss_parameters *params)

{
	int ret;
	ret = rdev->ops->change_bss(&rdev->wiphy, dev, params);
	return ret;
}

static inline int rdev_set_txq_params(struct cfg80211_registered_device *rdev,
				      struct net_device *dev,
				      struct ieee80211_txq_params *params)

{
	int ret;
	ret = rdev->ops->set_txq_params(&rdev->wiphy, dev, params);
	return ret;
}

static inline int
rdev_libertas_set_mesh_channel(struct cfg80211_registered_device *rdev,
			       struct net_device *dev,
			       struct ieee80211_channel *chan)
{
	int ret;
	ret = rdev->ops->libertas_set_mesh_channel(&rdev->wiphy, dev, chan);
	return ret;
}

static inline int
rdev_set_monitor_channel(struct cfg80211_registered_device *rdev,
			 struct cfg80211_chan_def *chandef)
{
	int ret;
	ret = rdev->ops->set_monitor_channel(&rdev->wiphy, chandef);
	return ret;
}

static inline int rdev_scan(struct cfg80211_registered_device *rdev,
			    struct cfg80211_scan_request *request)
{
	int ret;
	ret = rdev->ops->scan(&rdev->wiphy, request);
	return ret;
}

static inline void rdev_abort_scan(struct cfg80211_registered_device *rdev,
				   struct wireless_dev *wdev)
{
	rdev->ops->abort_scan(&rdev->wiphy, wdev);
}

static inline int rdev_auth(struct cfg80211_registered_device *rdev,
			    struct net_device *dev,
			    struct cfg80211_auth_request *req)
{
	int ret;
	ret = rdev->ops->auth(&rdev->wiphy, dev, req);
	return ret;
}

static inline int rdev_assoc(struct cfg80211_registered_device *rdev,
			     struct net_device *dev,
			     struct cfg80211_assoc_request *req)
{
	int ret;
	ret = rdev->ops->assoc(&rdev->wiphy, dev, req);
	return ret;
}

static inline int rdev_deauth(struct cfg80211_registered_device *rdev,
			      struct net_device *dev,
			      struct cfg80211_deauth_request *req)
{
	int ret;
	ret = rdev->ops->deauth(&rdev->wiphy, dev, req);
	return ret;
}

static inline int rdev_disassoc(struct cfg80211_registered_device *rdev,
				struct net_device *dev,
				struct cfg80211_disassoc_request *req)
{
	int ret;
	ret = rdev->ops->disassoc(&rdev->wiphy, dev, req);
	return ret;
}

static inline int rdev_connect(struct cfg80211_registered_device *rdev,
			       struct net_device *dev,
			       struct cfg80211_connect_params *sme)
{
	int ret;
	ret = rdev->ops->connect(&rdev->wiphy, dev, sme);
	return ret;
}

static inline int
rdev_update_connect_params(struct cfg80211_registered_device *rdev,
			   struct net_device *dev,
			   struct cfg80211_connect_params *sme, u32 changed)
{
	int ret;
	ret = rdev->ops->update_connect_params(&rdev->wiphy, dev, sme, changed);
	return ret;
}

static inline int rdev_disconnect(struct cfg80211_registered_device *rdev,
				  struct net_device *dev, u16 reason_code)
{
	int ret;
	ret = rdev->ops->disconnect(&rdev->wiphy, dev, reason_code);
	return ret;
}

static inline int rdev_join_ibss(struct cfg80211_registered_device *rdev,
				 struct net_device *dev,
				 struct cfg80211_ibss_params *params)
{
	int ret;
	ret = rdev->ops->join_ibss(&rdev->wiphy, dev, params);
	return ret;
}

static inline int rdev_leave_ibss(struct cfg80211_registered_device *rdev,
				  struct net_device *dev)
{
	int ret;
	ret = rdev->ops->leave_ibss(&rdev->wiphy, dev);
	return ret;
}

static inline int
rdev_set_wiphy_params(struct cfg80211_registered_device *rdev, u32 changed)
{
	int ret;

	if (!rdev->ops->set_wiphy_params)
		return -EOPNOTSUPP;

	ret = rdev->ops->set_wiphy_params(&rdev->wiphy, changed);
	return ret;
}

static inline int rdev_set_tx_power(struct cfg80211_registered_device *rdev,
				    struct wireless_dev *wdev,
				    enum nl80211_tx_power_setting type, int mbm)
{
	int ret;
	ret = rdev->ops->set_tx_power(&rdev->wiphy, wdev, type, mbm);
	return ret;
}

static inline int rdev_get_tx_power(struct cfg80211_registered_device *rdev,
				    struct wireless_dev *wdev, int *dbm)
{
	int ret;
	ret = rdev->ops->get_tx_power(&rdev->wiphy, wdev, dbm);
	return ret;
}

static inline int rdev_set_wds_peer(struct cfg80211_registered_device *rdev,
				    struct net_device *dev, const u8 *addr)
{
	int ret;
	ret = rdev->ops->set_wds_peer(&rdev->wiphy, dev, addr);
	return ret;
}

static inline int
rdev_set_multicast_to_unicast(struct cfg80211_registered_device *rdev,
			      struct net_device *dev,
			      const bool enabled)
{
	int ret;
	ret = rdev->ops->set_multicast_to_unicast(&rdev->wiphy, dev, enabled);
	return ret;
}

static inline void rdev_rfkill_poll(struct cfg80211_registered_device *rdev)
{
	rdev->ops->rfkill_poll(&rdev->wiphy);
}


#ifdef CONFIG_NL80211_TESTMODE
static inline int rdev_testmode_cmd(struct cfg80211_registered_device *rdev,
				    struct wireless_dev *wdev,
				    void *data, int len)
{
	int ret;
	ret = rdev->ops->testmode_cmd(&rdev->wiphy, wdev, data, len);
	return ret;
}

static inline int rdev_testmode_dump(struct cfg80211_registered_device *rdev,
				     struct sk_buff *skb,
				     struct netlink_callback *cb, void *data,
				     int len)
{
	int ret;

	ret = rdev->ops->testmode_dump(&rdev->wiphy, skb, cb, data, len);
	return ret;
}
#endif

static inline int
rdev_set_bitrate_mask(struct cfg80211_registered_device *rdev,
		      struct net_device *dev, const u8 *peer,
		      const struct cfg80211_bitrate_mask *mask)
{
	int ret;
	ret = rdev->ops->set_bitrate_mask(&rdev->wiphy, dev, peer, mask);
	return ret;
}

static inline int rdev_dump_survey(struct cfg80211_registered_device *rdev,
				   struct net_device *netdev, int idx,
				   struct survey_info *info)
{
	int ret;
	ret = rdev->ops->dump_survey(&rdev->wiphy, netdev, idx, info);
	return ret;
}

static inline int rdev_set_pmksa(struct cfg80211_registered_device *rdev,
				 struct net_device *netdev,
				 struct cfg80211_pmksa *pmksa)
{
	int ret;
	ret = rdev->ops->set_pmksa(&rdev->wiphy, netdev, pmksa);
	return ret;
}

static inline int rdev_del_pmksa(struct cfg80211_registered_device *rdev,
				 struct net_device *netdev,
				 struct cfg80211_pmksa *pmksa)
{
	int ret;
	ret = rdev->ops->del_pmksa(&rdev->wiphy, netdev, pmksa);
	return ret;
}

static inline int rdev_flush_pmksa(struct cfg80211_registered_device *rdev,
				   struct net_device *netdev)
{
	int ret;
	ret = rdev->ops->flush_pmksa(&rdev->wiphy, netdev);
	return ret;
}

static inline int
rdev_remain_on_channel(struct cfg80211_registered_device *rdev,
		       struct wireless_dev *wdev,
		       struct ieee80211_channel *chan,
		       unsigned int duration, u64 *cookie)
{
	int ret;
	ret = rdev->ops->remain_on_channel(&rdev->wiphy, wdev, chan,
					   duration, cookie);
	return ret;
}

static inline int
rdev_cancel_remain_on_channel(struct cfg80211_registered_device *rdev,
			      struct wireless_dev *wdev, u64 cookie)
{
	int ret;
	ret = rdev->ops->cancel_remain_on_channel(&rdev->wiphy, wdev, cookie);
	return ret;
}

static inline int rdev_mgmt_tx(struct cfg80211_registered_device *rdev,
			       struct wireless_dev *wdev,
			       struct cfg80211_mgmt_tx_params *params,
			       u64 *cookie)
{
	int ret;
	ret = rdev->ops->mgmt_tx(&rdev->wiphy, wdev, params, cookie);
	return ret;
}

static inline int
rdev_mgmt_tx_cancel_wait(struct cfg80211_registered_device *rdev,
			 struct wireless_dev *wdev, u64 cookie)
{
	int ret;
	ret = rdev->ops->mgmt_tx_cancel_wait(&rdev->wiphy, wdev, cookie);
	return ret;
}

static inline int rdev_set_power_mgmt(struct cfg80211_registered_device *rdev,
				      struct net_device *dev, bool enabled,
				      int timeout)
{
	int ret;
	ret = rdev->ops->set_power_mgmt(&rdev->wiphy, dev, enabled, timeout);
	return ret;
}

static inline int
rdev_set_cqm_rssi_config(struct cfg80211_registered_device *rdev,
			 struct net_device *dev, s32 rssi_thold, u32 rssi_hyst)
{
	int ret;
	ret = rdev->ops->set_cqm_rssi_config(&rdev->wiphy, dev, rssi_thold,
				       rssi_hyst);
	return ret;
}

static inline int
rdev_set_cqm_txe_config(struct cfg80211_registered_device *rdev,
			struct net_device *dev, u32 rate, u32 pkts, u32 intvl)
{
	int ret;
	ret = rdev->ops->set_cqm_txe_config(&rdev->wiphy, dev, rate, pkts,
					     intvl);
	return ret;
}

static inline void
rdev_mgmt_frame_register(struct cfg80211_registered_device *rdev,
			 struct wireless_dev *wdev, u16 frame_type, bool reg)
{
	might_sleep();

	rdev->ops->mgmt_frame_register(&rdev->wiphy, wdev , frame_type, reg);
}

static inline int rdev_set_antenna(struct cfg80211_registered_device *rdev,
				   u32 tx_ant, u32 rx_ant)
{
	int ret;
	ret = rdev->ops->set_antenna(&rdev->wiphy, tx_ant, rx_ant);
	return ret;
}

static inline int rdev_get_antenna(struct cfg80211_registered_device *rdev,
				   u32 *tx_ant, u32 *rx_ant)
{
	int ret;
	ret = rdev->ops->get_antenna(&rdev->wiphy, tx_ant, rx_ant);
	return ret;
}

static inline int
rdev_sched_scan_start(struct cfg80211_registered_device *rdev,
		      struct net_device *dev,
		      struct cfg80211_sched_scan_request *request)
{
	int ret;
	ret = rdev->ops->sched_scan_start(&rdev->wiphy, dev, request);
	return ret;
}

static inline int rdev_sched_scan_stop(struct cfg80211_registered_device *rdev,
				       struct net_device *dev)
{
	int ret;
	ret = rdev->ops->sched_scan_stop(&rdev->wiphy, dev);
	return ret;
}

static inline int rdev_set_rekey_data(struct cfg80211_registered_device *rdev,
				      struct net_device *dev,
				      struct cfg80211_gtk_rekey_data *data)
{
	int ret;
	ret = rdev->ops->set_rekey_data(&rdev->wiphy, dev, data);
	return ret;
}

static inline int rdev_tdls_mgmt(struct cfg80211_registered_device *rdev,
				 struct net_device *dev, u8 *peer,
				 u8 action_code, u8 dialog_token,
				 u16 status_code, u32 peer_capability,
				 bool initiator, const u8 *buf, size_t len)
{
	int ret;
	ret = rdev->ops->tdls_mgmt(&rdev->wiphy, dev, peer, action_code,
				   dialog_token, status_code, peer_capability,
				   initiator, buf, len);
	return ret;
}

static inline int rdev_tdls_oper(struct cfg80211_registered_device *rdev,
				 struct net_device *dev, u8 *peer,
				 enum nl80211_tdls_operation oper)
{
	int ret;
	ret = rdev->ops->tdls_oper(&rdev->wiphy, dev, peer, oper);
	return ret;
}

static inline int rdev_probe_client(struct cfg80211_registered_device *rdev,
				    struct net_device *dev, const u8 *peer,
				    u64 *cookie)
{
	int ret;
	ret = rdev->ops->probe_client(&rdev->wiphy, dev, peer, cookie);
	return ret;
}

static inline int rdev_set_noack_map(struct cfg80211_registered_device *rdev,
				     struct net_device *dev, u16 noack_map)
{
	int ret;
	ret = rdev->ops->set_noack_map(&rdev->wiphy, dev, noack_map);
	return ret;
}

static inline int
rdev_get_channel(struct cfg80211_registered_device *rdev,
		 struct wireless_dev *wdev,
		 struct cfg80211_chan_def *chandef)
{
	int ret;

	ret = rdev->ops->get_channel(&rdev->wiphy, wdev, chandef);

	return ret;
}

static inline int rdev_start_p2p_device(struct cfg80211_registered_device *rdev,
					struct wireless_dev *wdev)
{
	int ret;

	ret = rdev->ops->start_p2p_device(&rdev->wiphy, wdev);
	return ret;
}

static inline void rdev_stop_p2p_device(struct cfg80211_registered_device *rdev,
					struct wireless_dev *wdev)
{
	rdev->ops->stop_p2p_device(&rdev->wiphy, wdev);
}

static inline int rdev_start_nan(struct cfg80211_registered_device *rdev,
				 struct wireless_dev *wdev,
				 struct cfg80211_nan_conf *conf)
{
	int ret;

	ret = rdev->ops->start_nan(&rdev->wiphy, wdev, conf);
	return ret;
}

static inline void rdev_stop_nan(struct cfg80211_registered_device *rdev,
				 struct wireless_dev *wdev)
{
	rdev->ops->stop_nan(&rdev->wiphy, wdev);
}

static inline int
rdev_add_nan_func(struct cfg80211_registered_device *rdev,
		  struct wireless_dev *wdev,
		  struct cfg80211_nan_func *nan_func)
{
	int ret;

	ret = rdev->ops->add_nan_func(&rdev->wiphy, wdev, nan_func);
	return ret;
}

static inline void rdev_del_nan_func(struct cfg80211_registered_device *rdev,
				    struct wireless_dev *wdev, u64 cookie)
{
	rdev->ops->del_nan_func(&rdev->wiphy, wdev, cookie);
}

static inline int
rdev_nan_change_conf(struct cfg80211_registered_device *rdev,
		     struct wireless_dev *wdev,
		     struct cfg80211_nan_conf *conf, u32 changes)
{
	int ret;

	if (rdev->ops->nan_change_conf)
		ret = rdev->ops->nan_change_conf(&rdev->wiphy, wdev, conf,
						 changes);
	else
		ret = -ENOTSUPP;
	return ret;
}

static inline int rdev_set_mac_acl(struct cfg80211_registered_device *rdev,
				   struct net_device *dev,
				   struct cfg80211_acl_data *params)
{
	int ret;

	ret = rdev->ops->set_mac_acl(&rdev->wiphy, dev, params);
	return ret;
}

static inline int rdev_update_ft_ies(struct cfg80211_registered_device *rdev,
				     struct net_device *dev,
				     struct cfg80211_update_ft_ies_params *ftie)
{
	int ret;

	ret = rdev->ops->update_ft_ies(&rdev->wiphy, dev, ftie);
	return ret;
}

static inline int rdev_crit_proto_start(struct cfg80211_registered_device *rdev,
					struct wireless_dev *wdev,
					enum nl80211_crit_proto_id protocol,
					u16 duration)
{
	int ret;

	ret = rdev->ops->crit_proto_start(&rdev->wiphy, wdev,
					  protocol, duration);
	return ret;
}

static inline void rdev_crit_proto_stop(struct cfg80211_registered_device *rdev,
				       struct wireless_dev *wdev)
{
	rdev->ops->crit_proto_stop(&rdev->wiphy, wdev);
}

static inline int rdev_channel_switch(struct cfg80211_registered_device *rdev,
				      struct net_device *dev,
				      struct cfg80211_csa_settings *params)
{
	int ret;

	ret = rdev->ops->channel_switch(&rdev->wiphy, dev, params);
	return ret;
}

static inline int rdev_set_qos_map(struct cfg80211_registered_device *rdev,
				   struct net_device *dev,
				   struct cfg80211_qos_map *qos_map)
{
	int ret = -EOPNOTSUPP;

	if (rdev->ops->set_qos_map) {
		ret = rdev->ops->set_qos_map(&rdev->wiphy, dev, qos_map);
	}

	return ret;
}

static inline int
rdev_set_ap_chanwidth(struct cfg80211_registered_device *rdev,
		      struct net_device *dev, struct cfg80211_chan_def *chandef)
{
	int ret;

	ret = rdev->ops->set_ap_chanwidth(&rdev->wiphy, dev, chandef);

	return ret;
}

static inline int
rdev_add_tx_ts(struct cfg80211_registered_device *rdev,
	       struct net_device *dev, u8 tsid, const u8 *peer,
	       u8 user_prio, u16 admitted_time)
{
	int ret = -EOPNOTSUPP;

	if (rdev->ops->add_tx_ts)
		ret = rdev->ops->add_tx_ts(&rdev->wiphy, dev, tsid, peer,
					   user_prio, admitted_time);

	return ret;
}

static inline int
rdev_del_tx_ts(struct cfg80211_registered_device *rdev,
	       struct net_device *dev, u8 tsid, const u8 *peer)
{
	int ret = -EOPNOTSUPP;

	if (rdev->ops->del_tx_ts)
		ret = rdev->ops->del_tx_ts(&rdev->wiphy, dev, tsid, peer);

	return ret;
}

static inline int
rdev_tdls_channel_switch(struct cfg80211_registered_device *rdev,
			 struct net_device *dev, const u8 *addr,
			 u8 oper_class, struct cfg80211_chan_def *chandef)
{
	int ret;

	ret = rdev->ops->tdls_channel_switch(&rdev->wiphy, dev, addr,
					     oper_class, chandef);
	return ret;
}

static inline void
rdev_tdls_cancel_channel_switch(struct cfg80211_registered_device *rdev,
				struct net_device *dev, const u8 *addr)
{
	rdev->ops->tdls_cancel_channel_switch(&rdev->wiphy, dev, addr);
}

static inline int
rdev_start_radar_detection(struct cfg80211_registered_device *rdev,
			   struct net_device *dev,
			   struct cfg80211_chan_def *chandef,
			   u32 cac_time_ms)
{
	int ret = -ENOTSUPP;

	if (rdev->ops->start_radar_detection)
		ret = rdev->ops->start_radar_detection(&rdev->wiphy, dev,
						       chandef, cac_time_ms);
	return ret;
}

static inline int
rdev_set_mcast_rate(struct cfg80211_registered_device *rdev,
		    struct net_device *dev,
		    int mcast_rate[NUM_NL80211_BANDS])
{
	int ret = -ENOTSUPP;

		ret = rdev->ops->set_mcast_rate(&rdev->wiphy, dev, mcast_rate);
	return ret;
}

static inline int
rdev_set_coalesce(struct cfg80211_registered_device *rdev,
		  struct cfg80211_coalesce *coalesce)
{
	int ret = -ENOTSUPP;

	if (rdev->ops->set_coalesce)
		ret = rdev->ops->set_coalesce(&rdev->wiphy, coalesce);
	return ret;
}

static inline int
rdev_external_auth(struct cfg80211_registered_device *rdev,
		   struct net_device *dev,
		   struct cfg80211_external_auth_params *params)
{
	int ret = -EOPNOTSUPP;

	trace_rdev_external_auth(&rdev->wiphy, dev, params);
	if (rdev->ops->external_auth)
		ret = rdev->ops->external_auth(&rdev->wiphy, dev, params);
	trace_rdev_return_int(&rdev->wiphy, ret);
	return ret;
}

static inline int rdev_update_owe_info(struct cfg80211_registered_device *rdev,
				       struct net_device *dev,
				       struct cfg80211_update_owe_info *oweinfo)
{
	int ret = -EOPNOTSUPP;

	trace_rdev_update_owe_info(&rdev->wiphy, dev, oweinfo);
	if (rdev->ops->update_owe_info)
		ret = rdev->ops->update_owe_info(&rdev->wiphy, dev, oweinfo);
	trace_rdev_return_int(&rdev->wiphy, ret);
	return ret;
}

#endif /* __CFG80211_RDEV_OPS */
