# - Install "network-manager"
# - Place this file in /etc/netplan/ and run "netplan apply"
network:
    version: 2
    renderer: NetworkManager
    ethernets:
      eth0:
        dhcp4: true
        optional: true
    wifis:
        wlan0:
            dhcp4: true
            optional: true
            #addresses: [10.5.141.1/24]
            access-points:
               #"Robot_ENAC":
                    #password: "robotcities"
                    #mode: ap
                "robot":
                    password: "farmingmars"
            routes:
              - to: 239.0.0.0/4
                scope: link
                    #dhcp4: true
