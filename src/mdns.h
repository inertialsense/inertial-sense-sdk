//
// Created by firiusfoxx on 7/14/25.
//

#ifndef IS_SDK__MDNS_H
#define IS_SDK__MDNS_H

#define MDNS_RECORD_TIMEOUT_MS 1000 // Max time that any record will last for
#define MDNS_REQUEST_TIMEOUT_MS 200 // How long to wait for a response

#include <chrono>
#include <functional>
#include <random>
#include <unordered_map>
#include <shared_mutex>
#include <string>
#include <list>
#include <set>
#include <utility>
#include "libmdns/mdns.h"

class mdns {
public:
    // Delete the constructor as this is a static class
    mdns() = delete;

    // Define structures
    struct mdns_record_srv_cpp_t {
        uint16_t priority;
        uint16_t weight;
        uint16_t port;
        std::string name;

        mdns_record_srv_cpp_t() : mdns_record_srv_cpp_t(0,0,0,"") {} // Delegate to the other constructor.
        explicit mdns_record_srv_cpp_t(uint16_t a, uint16_t b, uint16_t c, std::string d) {
            priority = a;
            weight = b;
            port = c;
            name = std::move(d);
        }
        bool operator==(const mdns_record_srv_cpp_t &other) const = default;
        void swap(mdns_record_srv_cpp_t &other) {
            uint16_t placeholder = priority;
            priority = other.priority;
            other.priority = placeholder;

            placeholder = weight;
            weight = other.weight;
            other.weight = placeholder;

            placeholder = port;
            port = other.port;
            other.port = placeholder;

            name.swap(other.name);
        }
    };
    typedef struct mdns_record_srv_cpp_t mdns_record_srv_cpp_t;

    struct mdns_record_ptr_cpp_t {
        std::string name;

        mdns_record_ptr_cpp_t() : mdns_record_ptr_cpp_t("") {} // Delegate to the other constructor.
        explicit mdns_record_ptr_cpp_t(std::string a) {
            name = std::move(a);
        }
        bool operator==(const mdns_record_ptr_cpp_t &other) const = default;
        void swap(mdns_record_ptr_cpp_t &other) {
            name.swap(other.name);
        }
    };
    typedef struct mdns_record_ptr_cpp_t mdns_record_ptr_cpp_t;

    struct mdns_record_a_cpp_t {
        struct sockaddr_in addr;

        bool operator==(const mdns_record_a_cpp_t &other) const {
            if (addr.sin_family != other.addr.sin_family) return false;
            if (addr.sin_port != other.addr.sin_port) return false;
            if (addr.sin_addr.s_addr != other.addr.sin_addr.s_addr) return false;
            return true;
        };
        void swap(mdns_record_a_cpp_t &other) {
            in_addr_t placeholderAddr = addr.sin_addr.s_addr;
            addr.sin_addr.s_addr = other.addr.sin_addr.s_addr;
            other.addr.sin_addr.s_addr = placeholderAddr;

            in_port_t placeholderPort = addr.sin_port;
            addr.sin_port = other.addr.sin_port;
            other.addr.sin_port = placeholderPort;

            sa_family_t placeholderFamily = addr.sin_family;
            addr.sin_family = other.addr.sin_family;
            other.addr.sin_family = placeholderFamily;
        }

        mdns_record_a_cpp_t() = default;
        explicit mdns_record_a_cpp_t(struct sockaddr_in addr) {
            this->addr = addr;
        };
    };
    typedef struct mdns_record_a_cpp_t mdns_record_a_cpp_t;

    struct mdns_record_aaaa_cpp_t {
        struct sockaddr_in6 addr;

        bool operator==(const mdns_record_aaaa_cpp_t &other) const {
            if (addr.sin6_flowinfo != other.addr.sin6_flowinfo) return false;
            if (addr.sin6_scope_id != other.addr.sin6_scope_id) return false;
            if (addr.sin6_family != other.addr.sin6_family) return false;
            if (addr.sin6_port != other.addr.sin6_port) return false;
            for (int i = 0; i < 16; ++i) {
                if (addr.sin6_addr.s6_addr[i] != other.addr.sin6_addr.s6_addr[i]) return false;
            }
            return true;
        };
        void swap(mdns_record_aaaa_cpp_t &other) {
            uint32_t placeholderFlowinfo = addr.sin6_flowinfo;
            addr.sin6_flowinfo = other.addr.sin6_flowinfo;
            other.addr.sin6_flowinfo = placeholderFlowinfo;

            uint32_t placeholderScopeId = addr.sin6_scope_id;
            addr.sin6_scope_id = other.addr.sin6_scope_id;
            other.addr.sin6_scope_id = placeholderScopeId;

            sa_family_t placeholderFamily = addr.sin6_family;
            addr.sin6_family = other.addr.sin6_family;
            other.addr.sin6_family = placeholderFamily;

            in_port_t placeholderPort = addr.sin6_port;
            addr.sin6_port = other.addr.sin6_port;
            other.addr.sin6_port = placeholderPort;

            for (int i = 0; i < 16; ++i) {
                uint8_t placeholderAddr;
                placeholderAddr = addr.sin6_addr.s6_addr[i];
                addr.sin6_addr.s6_addr[i] = other.addr.sin6_addr.s6_addr[i];
                other.addr.sin6_addr.s6_addr[i] = placeholderAddr;
            }
        }

        mdns_record_aaaa_cpp_t() = default;
        explicit mdns_record_aaaa_cpp_t(const struct sockaddr_in6& addr) {
            this->addr = addr;
        };
    };
    typedef struct mdns_record_aaaa_cpp_t mdns_record_aaaa_cpp_t;

    struct mdns_record_txt_cpp_t {
        std::string key;
        std::string value;

        mdns_record_txt_cpp_t() : mdns_record_txt_cpp_t("", "") {} // Delegate to the other constructor.
        explicit mdns_record_txt_cpp_t(std::string a, std::string b) {
            key = std::move(a);
            value = std::move(b);
        }
        bool operator==(const mdns_record_txt_cpp_t &other) const = default;
        void swap(mdns_record_txt_cpp_t &other) {
            key.swap(other.key);
            value.swap(other.value);
        }
    };
    typedef struct mdns_record_txt_cpp_t mdns_record_txt_cpp_t;

    struct mdns_record_cpp_t {
        std::string name;
        mdns_record_type_t type{MDNS_RECORDTYPE_IGNORE};
        union mdns_record_data {
            mdns_record_ptr_cpp_t ptr;
            mdns_record_srv_cpp_t srv;
            mdns_record_a_cpp_t a{};
            mdns_record_aaaa_cpp_t aaaa;
            mdns_record_txt_cpp_t txt;

            mdns_record_data() {memset((void*)this,0,sizeof(mdns_record_data));}
            ~mdns_record_data() {;}
        } data;
        uint16_t rclass{};
        uint32_t ttl{};

        ~mdns_record_cpp_t() {
            switch (type) {
                case MDNS_RECORDTYPE_IGNORE: break;
                case MDNS_RECORDTYPE_A: data.a.~mdns_record_a_cpp_t(); break;
                case MDNS_RECORDTYPE_TXT: data.txt.~mdns_record_txt_cpp_t(); break;
                case MDNS_RECORDTYPE_PTR: data.ptr.~mdns_record_ptr_cpp_t(); break;
                case MDNS_RECORDTYPE_AAAA: data.aaaa.~mdns_record_aaaa_cpp_t(); break;
                //case MDNS_RECORDTYPE_SRV: data.srv.~mdns_record_srv_cpp_t(); break; // I don't know why this isn't needed for SRV records but needed for all the other records but calling it causes segfaults with a double free ¯\_(ツ)_/¯
                case MDNS_RECORDTYPE_ANY: break;
            }
        }
        mdns_record_cpp_t(const mdns_record_cpp_t &other) {
            name = other.name;
            type = other.type;
            rclass = other.rclass;
            ttl = other.ttl;
            switch (type) {
                case MDNS_RECORDTYPE_IGNORE: break;
                case MDNS_RECORDTYPE_A: data.a = other.data.a; break;
                case MDNS_RECORDTYPE_TXT: data.txt = other.data.txt; break;
                case MDNS_RECORDTYPE_PTR: data.ptr = other.data.ptr; break;
                case MDNS_RECORDTYPE_AAAA: data.aaaa = other.data.aaaa; break;
                case MDNS_RECORDTYPE_SRV: data.srv = other.data.srv; break;
                case MDNS_RECORDTYPE_ANY: break;
            }
        }
        mdns_record_cpp_t() = default;
        bool operator==(const mdns_record_cpp_t &other) const {
            if ((name != other.name) || (type != other.type) || (rclass != other.rclass) || (ttl != other.ttl)) return false;
            if (type == MDNS_RECORDTYPE_PTR) {
                return data.ptr == other.data.ptr;
            } else if (type == MDNS_RECORDTYPE_SRV) {
                return data.srv == other.data.srv;
            } else if (type == MDNS_RECORDTYPE_A) {
                return data.a == other.data.a;
            } else if (type == MDNS_RECORDTYPE_AAAA) {
                return data.aaaa == other.data.aaaa;
            } else if (type == MDNS_RECORDTYPE_TXT) {
                return data.txt == other.data.txt;
            } else {
                return true; // ANY and IGNORE record types don't have data to compare
            }
        };
        void swap(mdns_record_cpp_t &other) {
            name.swap(other.name);

            mdns_record_type_t typePlaceholder = type;
            type = other.type;
            other.type = typePlaceholder;

            uint16_t rclassPlaceholder = rclass;
            rclass = other.rclass;
            other.rclass = rclassPlaceholder;

            uint16_t ttlPlaceholder = ttl;
            ttl = other.ttl;
            other.ttl = ttlPlaceholder;

            if (type == MDNS_RECORDTYPE_PTR) {
                data.ptr.swap(other.data.ptr);
            } else if (type == MDNS_RECORDTYPE_SRV) {
                data.srv.swap(other.data.srv);
            } else if (type == MDNS_RECORDTYPE_A) {
                data.a.swap(other.data.a);
            } else if (type == MDNS_RECORDTYPE_AAAA) {
                data.aaaa.swap(other.data.aaaa);
            } else if (type == MDNS_RECORDTYPE_TXT) {
                data.txt.swap(other.data.txt);
            }
        }
        mdns_record_cpp_t& operator=(mdns_record_cpp_t other) {
            swap(other);
            return *this;
        }

    };
    typedef struct mdns_record_cpp_t mdns_record_cpp_t;

    // We need to write our own hash function to store mdns records in a hashmap
    struct mdns_record_cpp_tHash {
        std::size_t operator()(const mdns::mdns_record_cpp_t& r) const noexcept {
            std::size_t hash1 = std::hash<std::string>{}(std::to_string(r.ttl));
            std::size_t hash2 = std::hash<std::string>{}(std::to_string(r.rclass));
            hash1 = hash1 ^ (hash2 << 1); // Combine hashes
            hash2 = std::hash<std::string>{}(std::to_string(r.type));
            hash1 = hash1 ^ (hash2 << 1);
            hash2 = std::hash<std::string>{}(r.name);
            hash1 = hash1 ^ (hash2 << 1);
            if (r.type == MDNS_RECORDTYPE_PTR) {
                hash2 = std::hash<std::string>{}(r.data.ptr.name);
                hash1 = hash1 ^ (hash2 << 1);
            } else if (r.type == MDNS_RECORDTYPE_SRV) {
                hash2 = std::hash<std::string>{}(std::to_string(r.data.srv.weight));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.srv.port));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.srv.priority));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(r.data.srv.name);
                hash1 = hash1 ^ (hash2 << 1);
            } else if (r.type == MDNS_RECORDTYPE_A) {
                hash2 = std::hash<std::string>{}(std::to_string(r.data.a.addr.sin_port));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.a.addr.sin_family));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.a.addr.sin_addr.s_addr));
                hash1 = hash1 ^ (hash2 << 1);
            } else if (r.type == MDNS_RECORDTYPE_AAAA) {
                hash2 = std::hash<std::string>{}(std::to_string(r.data.aaaa.addr.sin6_port));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.aaaa.addr.sin6_scope_id));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.aaaa.addr.sin6_flowinfo));
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(std::to_string(r.data.aaaa.addr.sin6_family));
                hash1 = hash1 ^ (hash2 << 1);
                for (int i = 0; i < 16; ++i) {
                    hash2 = std::hash<std::string>{}(std::to_string(r.data.aaaa.addr.sin6_addr.s6_addr[i]));
                    hash1 = hash1 ^ (hash2 << 1);
                }
            } else if (r.type == MDNS_RECORDTYPE_TXT) {
                hash2 = std::hash<std::string>{}(r.data.txt.value);
                hash1 = hash1 ^ (hash2 << 1);
                hash2 = std::hash<std::string>{}(r.data.txt.key);
                hash1 = hash1 ^ (hash2 << 1);
            }
            return hash1;
        }
    };

    // Public functions
    static void tick();
    static void sendQuery(mdns_record_type_t type, const std::string& query);
    static std::vector<mdns::mdns_record_cpp_t> getRecords(const std::function<bool(mdns_record_cpp_t)>& filter);
    static std::vector<mdns_record_cpp_t> getRecords();
    static sockaddr_storage resolveName(const std::string& name);

private:
    // Mutex
    inline static std::shared_mutex mutex;

    // Information related to sockets
    inline static int mdnsSockets[32];
    inline static int socketsOpened;

    // Random Number generator
    inline static std::random_device hwRandom;
    inline static int backtick;

    // Structs
    typedef struct {
        uint16_t queryId; // Query Id
        std::chrono::time_point<std::chrono::steady_clock> querySent; // Time when Query got sent
        bool queryRecieved; // If this query got received and this entry should be deleted
    } used_query_id_t;

    // Lists
    inline static std::list<used_query_id_t> usedQueryIds;
    inline static std::unordered_map<mdns_record_cpp_t, std::chrono::time_point<std::chrono::steady_clock>, mdns_record_cpp_tHash> responses;

    // Functions
    static int createMdnsSockets();
    static int queryCallback(int sock, const struct sockaddr* from, size_t addrlen, mdns_entry_type_t entry, uint16_t query_id,
                             uint16_t rtype, uint16_t rclass, uint32_t ttl, const void* data, size_t size, size_t name_offset,
                             size_t name_length, size_t record_offset, size_t record_length, void* user_data);
    static int handleMdnsQueryResponses();
    static void cleanupExpiredResponses();
};

#endif //IS_SDK__MDNS_H
