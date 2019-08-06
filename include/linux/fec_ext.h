#ifndef FEC_EXT_H
#define FEC_EXT_H

typedef void (*tx_start_fnc)(struct sk_buff* skb, void* user_data);
typedef void (*tx_complete_fnc)(struct sk_buff* skb, void* user_data);
typedef void (*rx_complete_fnc)(struct sk_buff* skb, void* user_data);

struct fec_ext_callbacks
{
	tx_start_fnc tx_start;
	tx_complete_fnc tx_complete;
	rx_complete_fnc rx_complete;
	void* user_data;
};

void fec_ext_callbacks(struct net_device *ndev, struct fec_ext_callbacks* cbs);

#endif
