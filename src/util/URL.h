//
// Created by firiusfoxx on 2025-08-05.
//

#ifndef IS_SDK___URL_H
#define IS_SDK___URL_H

#include <curl/curl.h>
#include <util.h>
using namespace utils;

class URL {
public:
    URL(const std::string& urlStr);

    // Getters for private variables
    std::string getUrl();
    std::string getUrlPunycode();
    std::string getUrlIDN();
    std::string getScheme();
    std::string getUser();
    std::string getPassword();
    std::string getHost();
    std::string getHostPunycode();
    std::string getHostIDN();
    std::string getZoneid();
    std::string getPort();
    std::string getPath();
    std::string getQuery();
    std::string getFragment();

    // Derives a canonical host for passing to getaddrinfo (removes square brackets and adds zoneid)
    std::string getHostCanonized();
private:
    static std::unique_ptr<std::string> parseURLPart(const CURLU *url, CURLUPart part, unsigned int flags, const std::string& exceptionMsg);
    static std::string getURLPart(const std::unique_ptr<std::string> &part, const std::string& noValueExceptionMsg);

    std::unique_ptr<std::string> url; // What the user actually entered
    std::unique_ptr<std::string> urlPunycode; // Used to actually resolve the URL
    std::unique_ptr<std::string> urlIDN; // Used to render the URL
    std::unique_ptr<std::string> scheme;
    std::unique_ptr<std::string> user;
    std::unique_ptr<std::string> password;
    std::unique_ptr<std::string> host; // What the user actually entered
    std::unique_ptr<std::string> hostPunycode; // Used to actually resolve the URL
    std::unique_ptr<std::string> hostIDN; // Used to render the URL
    std::unique_ptr<std::string> zoneid;
    std::unique_ptr<std::string> port;
    std::unique_ptr<std::string> path;
    std::unique_ptr<std::string> query;
    std::unique_ptr<std::string> fragment;
};


#endif //IS_SDK___URL_H
