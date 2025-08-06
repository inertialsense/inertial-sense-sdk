//
// Created by firiusfoxx on 2025-08-05.
//

#include "URL.h"

#include <memory>

URL::URL(const std::string& urlStr) {
    CURLUcode rc;
    CURLU *curlUrl = curl_url();
    rc = curl_url_set(curlUrl, CURLUPART_URL, urlStr.c_str(), CURLU_NON_SUPPORT_SCHEME);
    if (!rc) {
        try {
            url = parseURLPart(curlUrl, CURLUPART_URL, 0, "Invalid URL (This shouldn't happen)");
            urlPunycode = parseURLPart(curlUrl, CURLUPART_URL, CURLU_PUNYCODE, "Failed to convert URL to Punycode");
            urlIDN = parseURLPart(curlUrl, CURLUPART_URL, CURLU_PUNY2IDN, "Failed to convert URL to IDN");
            scheme = parseURLPart(curlUrl, CURLUPART_SCHEME, 0, "Failed to parse scheme");
            user = parseURLPart(curlUrl, CURLUPART_USER, 0, "Failed to parse username");
            password = parseURLPart(curlUrl, CURLUPART_PASSWORD, 0, "Failed to parse password");
            host = parseURLPart(curlUrl, CURLUPART_HOST, 0, "Failed to parse host");
            hostPunycode = parseURLPart(curlUrl, CURLUPART_HOST, CURLU_PUNYCODE, "Failed to convert host to Punycode");
            hostIDN = parseURLPart(curlUrl, CURLUPART_HOST, CURLU_PUNY2IDN, "Failed to convert host to IDN");
            zoneid = parseURLPart(curlUrl, CURLUPART_ZONEID, 0, "Failed to parse IPv6 ZoneID");
            port = parseURLPart(curlUrl, CURLUPART_PORT, 0, "Failed to parse port");
            path = parseURLPart(curlUrl, CURLUPART_PATH, 0, "Failed to parse path");
            query = parseURLPart(curlUrl, CURLUPART_QUERY, 0, "Failed to parse query");
            fragment = parseURLPart(curlUrl, CURLUPART_FRAGMENT, 0, "Failed to parse fragment");
        } catch (std::invalid_argument &e) {
            curl_url_cleanup(curlUrl);
            throw e;
        }
    }
    curl_url_cleanup(curlUrl);
    if (rc) throw std::invalid_argument("Invalid URL");
}

std::unique_ptr<std::string> URL::parseURLPart(const CURLU *url, CURLUPart part, unsigned int flags, const std::string& exceptionMsg) {
    CURLUcode rc;
    char *urlPart;
    std::unique_ptr<std::string> returnValue = std::unique_ptr<std::string>(nullptr);
    rc = curl_url_get(url, part, &urlPart, flags);
    if (rc && rc != CURLUE_NO_USER && rc != CURLUE_NO_PASSWORD && rc != CURLUE_NO_OPTIONS && rc != CURLUE_NO_PORT
           && rc != CURLUE_NO_QUERY && rc != CURLUE_NO_FRAGMENT && rc != CURLUE_NO_ZONEID && rc != CURLUE_LACKS_IDN)
        throw std::invalid_argument(exceptionMsg);
    if (urlPart != nullptr) returnValue = std::make_unique<std::string>(urlPart);
    curl_free(urlPart);
    return returnValue;
}

std::string URL::getURLPart(const std::unique_ptr<std::string> &part, const std::string& noValueExceptionMsg) {
    if (part) {
        return std::string{*part};
    }
    throw std::domain_error(noValueExceptionMsg);
}

std::string URL::getUrl() { return getURLPart(url, "Failed to get URL"); }
std::string URL::getUrlPunycode() { return getURLPart(urlPunycode, "Failed to get Punycode URL"); }
std::string URL::getUrlIDN() { return getURLPart(urlIDN, "Failed to get IDN URL"); }
std::string URL::getScheme() { return getURLPart(scheme, "No scheme specified in URL"); }
std::string URL::getUser() { return getURLPart(user, "No user specified in URL"); }
std::string URL::getPassword() { return getURLPart(password, "No password specified in URL"); }
std::string URL::getHost() { return getURLPart(host, "No host specified in URL"); }
std::string URL::getHostPunycode() { return getURLPart(hostPunycode, "Failed to get IDN URL"); }
std::string URL::getHostIDN() { return getURLPart(hostIDN, "Failed to get IDN URL"); }
std::string URL::getZoneid() { return getURLPart(zoneid, "Failed to get IDN URL"); }
std::string URL::getPort() { return getURLPart(port, "No port in URL"); }
std::string URL::getPath() { return getURLPart(path, "Failed to get URL Path"); }
std::string URL::getQuery() { return getURLPart(query, "No query in URL"); }
std::string URL::getFragment() { return getURLPart(fragment, "No fragment specified URL"); }

std::string URL::getHostCanonized() {
    std::string returnValue;
    try {
        returnValue = getHostPunycode();
    } catch (std::domain_error &e) {
        returnValue = getHost();
    }
    if (returnValue.starts_with('[') && returnValue.ends_with(']')) {
        returnValue = returnValue.substr(1, returnValue.length() - 2);
        try {
            std::string zoneId = getZoneid();
            if (!zoneId.empty()) {
                returnValue = returnValue.append("%");
                returnValue = returnValue.append(zoneId);
            }
        } catch (std::domain_error &e) {}
    }
    return returnValue;
}

