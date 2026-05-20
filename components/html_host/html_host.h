#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#ifdef USE_WEBSERVER
#include "esphome/components/web_server_base/web_server_base.h"
#if !USE_ESP32
#include <ESPAsyncWebServer.h>
#endif
#endif

namespace esphome {
namespace html_host {

class HtmlHostComponent : public Component {
 public:
  HtmlHostComponent(const std::string &url, const std::string &content_type, const uint8_t *html_data, size_t html_size)
      : url_(url), content_type_(content_type), html_data_(html_data), html_size_(html_size) {}

  void setup() override {
#ifdef USE_WEBSERVER
    if (web_server_base::global_web_server_base != nullptr) {
      class HtmlHostHandler : public AsyncWebHandler {
       protected:
        HtmlHostComponent *parent_;

       public:
        HtmlHostHandler(HtmlHostComponent *parent) : parent_(parent) {}

        bool canHandle(AsyncWebServerRequest *request) const override {
          if (request->method() != HTTP_GET)
            return false;
          char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
          std::string request_url = request->url_to(url_buf);
          return request_url == parent_->get_url();
        }

        void handleRequest(AsyncWebServerRequest *request) override {
          std::string html(reinterpret_cast<const char *>(parent_->get_html_data()),
                           parent_->get_html_size());
          AsyncWebServerResponse *response =
              request->beginResponse(200, parent_->get_content_type().c_str(), html);
          response->addHeader("Content-Encoding", "gzip");
          request->send(response);
        }
      };

      web_server_base::global_web_server_base->add_handler(new HtmlHostHandler(this));
    }
#endif
  }

  float get_setup_priority() const override { return setup_priority::WIFI - 1.0f; }

  const std::string &get_url() const { return this->url_; }
  const std::string &get_content_type() const { return this->content_type_; }
  const uint8_t *get_html_data() const { return this->html_data_; }
  size_t get_html_size() const { return this->html_size_; }

 protected:
  std::string url_;
  std::string content_type_;
  const uint8_t *html_data_;
  size_t html_size_;
};

}  // namespace html_host
}  // namespace esphome
