#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509.h>
#include <openssl/err.h>
#include <openssl/bio.h>

using json = nlohmann::json;

class CloudActivator
{
public:
    // 构造函数：传入云端(或本地测试机)URL，公钥路径，授权文件保存路径
    CloudActivator(const std::string &server_url,
                   const std::string &cert_path,
                   const std::string &lic_path)
        : _server_url(server_url), _cert_path(cert_path), _lic_path(lic_path) {}

    // ---------------------------------------------------------
    // 功能1：向云端请求激活（首次激活时调用）
    // ---------------------------------------------------------
    bool activateDevice(const std::string &activation_key, const std::string &iface = "eth0")
    {
        std::string mac = getMacAddress(iface);
        if (mac.empty())
        {
            std::cerr << "[Error] 无法获取网卡 " << iface << " 的 MAC 地址!" << std::endl;
            return false;
        }

        std::cout << "[Info] 正在向服务器请求激活, MAC: " << mac << std::endl;

        CURL *curl = curl_easy_init();
        if (!curl)
            return false;

        std::string readBuffer;
        json request_data;
        request_data["mac_address"] = mac;
        request_data["activation_key"] = activation_key;
        std::string json_str = request_data.dump();

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, _server_url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);

        if (res != CURLE_OK)
        {
            std::cerr << "[Error] 网络请求失败: " << curl_easy_strerror(res) << std::endl;
            return false;
        }

        try
        {
            auto response = json::parse(readBuffer);
            if (response["status"] == "success")
            {
                // 将服务器返回的 JSON 内容格式化后保存为本地 .lic 文件
                std::ofstream out(_lic_path);
                out << response["license_content"].dump(4);
                out.close();
                std::cout << "[Success] 激活成功！授权文件已保存至: " << _lic_path << std::endl;
                return true;
            }
            else
            {
                std::cerr << "[Error] 激活失败: " << response["detail"] << std::endl;
                return false;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Error] 解析服务器响应失败: " << e.what() << std::endl;
            return false;
        }
    }

    // ---------------------------------------------------------
    // 功能2：每次机器人程序启动时的强制本地校验 (离线运行)
    // ---------------------------------------------------------
    bool verifyLocalLicense(const std::string &iface = "eth0")
    {
        std::string local_mac = getMacAddress(iface);
        if (local_mac.empty())
            return false;

        std::ifstream lic_file(_lic_path);
        if (!lic_file.is_open())
        {
            std::cerr << "[Fatal] 找不到授权文件: " << _lic_path << "，请先联网激活。" << std::endl;
            return false;
        }

        json lic_json;
        try
        {
            lic_file >> lic_json;
        }
        catch (...)
        {
            std::cerr << "[Fatal] 授权文件格式损坏!" << std::endl;
            return false;
        }

        std::string payload_str = lic_json["payload"];
        std::string signature_b64 = lic_json["signature"];

        // 1. 密码学验签 (防篡改)
        if (!verifySignature(payload_str, signature_b64))
        {
            std::cerr << "[Fatal] 数字签名校验失败！授权文件被非法篡改！" << std::endl;
            return false;
        }

        // 2. 解析明文载荷，进行逻辑匹配
        json payload = json::parse(payload_str);
        std::string bind_mac = payload["mac"];
        long expiry_timestamp = payload["expiry"];

        // 校验 MAC 地址是否匹配本机
        // 宇树机器人的 MAC 有时因为网卡驱动会变成全大写或全小写，建议统一转大写比较
        if (toUpperCase(local_mac) != toUpperCase(bind_mac))
        {
            std::cerr << "[Fatal] 硬件指纹不匹配！此程序被非法拷贝至其他机器人。" << std::endl;
            return false;
        }

        // 校验是否过期
        long current_timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();
        if (current_timestamp > expiry_timestamp)
        {
            std::cerr << "[Fatal] 授权已过期！请联系千二科技续费。" << std::endl;
            return false;
        }

        std::cout << "[Success] 千二科技正版授权验证通过。允许运行。" << std::endl;
        return true;
    }

private:
    std::string _server_url;
    std::string _cert_path;
    std::string _lic_path;

    // Curl 写回调
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *userp)
    {
        userp->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // 获取 MAC 地址
    std::string getMacAddress(const std::string &iface)
    {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0)
            return "";
        struct ifreq ifr;
        strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0)
        {
            unsigned char mac[6];
            memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            close(fd);
            return std::string(mac_str);
        }
        close(fd);
        return "";
    }

    // 字符串转大写工具
    std::string toUpperCase(std::string s)
    {
        for (char &c : s)
            c = toupper(c);
        return s;
    }

    // Base64 解码 (利用 OpenSSL BIO)
    std::vector<unsigned char> base64Decode(const std::string &b64_str)
    {
        BIO *bio = BIO_new_mem_buf(b64_str.data(), b64_str.size());
        BIO *b64 = BIO_new(BIO_f_base64());
        bio = BIO_push(b64, bio);
        BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL); // 忽略换行符

        std::vector<unsigned char> decoded(b64_str.size());
        int decoded_length = BIO_read(bio, decoded.data(), b64_str.size());
        BIO_free_all(bio);

        if (decoded_length > 0)
            decoded.resize(decoded_length);
        else
            decoded.clear();
        return decoded;
    }

    // 核心验签逻辑
    bool verifySignature(const std::string &payload, const std::string &signature_b64)
    {
        // 解码 Base64 签名
        std::vector<unsigned char> signature = base64Decode(signature_b64);
        if (signature.empty())
            return false;

        // 读取公钥 (ZJUDES.crt)
        FILE *cert_file = fopen(_cert_path.c_str(), "r");
        if (!cert_file)
        {
            std::cerr << "[Error] 找不到公钥证书: " << _cert_path << std::endl;
            return false;
        }

        X509 *cert = PEM_read_X509(cert_file, nullptr, nullptr, nullptr);
        fclose(cert_file);
        if (!cert)
            return false;

        EVP_PKEY *pubkey = X509_get_pubkey(cert);
        X509_free(cert);
        if (!pubkey)
            return false;

        // 初始化验签
        EVP_MD_CTX *ctx = EVP_MD_CTX_new();
        bool result = false;
        if (EVP_DigestVerifyInit(ctx, nullptr, EVP_sha256(), nullptr, pubkey) > 0)
        {
            if (EVP_DigestVerifyUpdate(ctx, payload.c_str(), payload.size()) > 0)
            {
                if (EVP_DigestVerifyFinal(ctx, signature.data(), signature.size()) == 1)
                {
                    result = true;
                }
            }
        }

        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pubkey);
        return result;
    }
};