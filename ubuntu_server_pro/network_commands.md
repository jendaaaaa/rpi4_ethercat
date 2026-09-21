# Raspberry Pi: nmcli essentials

For Raspberry Pi OS using NetworkManager. Replace `wlan0` with your interface and `MyWiFi` with your saved connection name (which may differ from the Wi-Fi SSID).

> Disconnecting, switching networks, or applying IP changes over Wi-Fi can drop your SSH session.

## Status and scanning

```bash
nmcli general status
nmcli device status
nmcli device wifi list ifname wlan0 --rescan yes
```

## Connect to a new Wi-Fi network

Prompts for the password, saves a profile, and connects:

```bash
sudo nmcli --ask device wifi connect "MyWiFi" ifname wlan0
```

## Saved and active connections

```bash
nmcli connection show
nmcli connection show --active
```

## Autoconnect and priority

```bash
sudo nmcli connection modify "MyWiFi" connection.autoconnect yes
sudo nmcli connection modify "MyWiFi" connection.autoconnect-priority 10
nmcli -f connection.autoconnect,connection.autoconnect-priority connection show "MyWiFi"
```

Higher priority wins when choosing among available autoconnect profiles; it does not switch an already active connection. Use `connection.autoconnect no` to disable automatic connection.

## Disconnect, reconnect, or forget

```bash
# Disconnect the interface and prevent automatic reconnection
sudo nmcli device disconnect wlan0

# Reconnect using a specific saved profile
sudo nmcli connection up "MyWiFi" ifname wlan0

# Delete a saved profile (also disconnects it if active)
sudo nmcli connection delete "MyWiFi"
```

## IP address, gateway, and DNS

```bash
nmcli -f IP4,IP6 device show wlan0
ip -brief address show wlan0
ip route show default
```

## DHCP or static IPv4

Set a static address. Adapt these examples to your LAN and choose an unused address outside the DHCP pool, or reserve it on your router:

```bash
sudo nmcli connection modify "MyWiFi" \
  ipv4.method manual \
  ipv4.addresses "192.168.1.50/24" \
  ipv4.gateway "192.168.1.1" \
  ipv4.dns "1.1.1.1 8.8.8.8"
```

Or return to DHCP and automatically supplied DNS:

```bash
sudo nmcli connection modify "MyWiFi" \
  ipv4.method auto \
  ipv4.addresses "" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.ignore-auto-dns no
```

Apply either change by reactivating the profile:

```bash
sudo nmcli connection up "MyWiFi"
```

## Minimal troubleshooting

```bash
sudo nmcli radio wifi on
rfkill list                         # Check for a blocked radio
sudo rfkill unblock wifi            # Clear a software block
systemctl is-active NetworkManager
sudo journalctl -u NetworkManager -b -n 50 --no-pager

ping -c 3 192.168.1.1                # Your actual gateway
ping -c 3 1.1.1.1                    # Internet reachability
getent hosts example.com            # DNS resolution
```

If the gateway responds but an internet IP does not, check upstream connectivity. If internet access works by IP but name lookup fails, check DNS. Some networks block ping.

Reference: [NetworkManager settings](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html) and [autoconnect behavior](https://www.networkmanager.dev/docs/api/latest/settings-connection.html).
