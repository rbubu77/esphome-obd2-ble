#include "elm327_ble.h"
#include "esphome/core/log.h"

namespace espbt = esphome::esp32_ble_tracker;

namespace esphome {
namespace elm327_ble {

static const char *TAG = "elm327_ble";

// ============================================================
// PATCH VW MEB (ID.3) - resume des modifications
// ============================================================
// 1. STATE_SWITCHING_HEADER passe de 2 a 4 etapes :
//        ATCRA<filtre>   filtre de reception (absent du composant d'origine)
//        ATSH<header>    header d'emission (deja present)
//        1003            session diagnostic etendue      <-- LA CLE
//        -> envoi de la requete
//    Sans le 1003, tous les DID mode 22 renvoient NODATA sur MEB.
//    Valide sur ID.3 : 22028C -> 62028CEF, 22295A -> 62295A0086FB.
//
// 2. La sequence est rejouee AVANT CHAQUE requete a header, pas seulement
//    au changement de header : la session expire en ~5 s cote vehicule.
//
// 3. Le filtre de reception est derive du header (convention VW : l'octet
//    FC de la requete devient FE dans la reponse). Aucun changement du
//    schema YAML n'est donc necessaire.
//
// 4. CORRECTION DE BUG dans dispatch_raw_pid_response() : le seuil
//    anti-troncature exigeait 8 caracteres de donnees, ce qui rejetait
//    silencieusement toute reponse courte. 62028CEF (1 octet utile) et
//    62295A0086FB (3 octets) etaient jetes. Seuil ramene a 1 octet.
//
// 5. .h : ajouter  bool header_ready_{false};  aux membres prives.
// ============================================================

// Derive l'adresse de reponse a partir de l'adresse de requete.
//   29 bits VW : 17FC007B -> 17FE007B  (FC devient FE)
//   11 bits    : 7E5      -> 7ED       (+8 sur le dernier nibble)
// Chaine vide = pas de filtre applicable.
static std::string derive_rx_filter(const std::string &header) {
  if (header.length() == 8) {
    if ((header[2] == 'F' || header[2] == 'f') &&
        (header[3] == 'C' || header[3] == 'c')) {
      std::string f = header;
      f[3] = 'E';
      return f;
    }
    return "";
  }
  if (header.length() == 3 && (header[0] == '7') &&
      (header[1] == 'E' || header[1] == 'e')) {
    char c = header[2];
    int n = -1;
    if (c >= '0' && c <= '9') n = c - '0';
    else if (c >= 'A' && c <= 'F') n = c - 'A' + 10;
    else if (c >= 'a' && c <= 'f') n = c - 'a' + 10;
    if (n >= 0 && n < 8) {
      std::string f = "7E";
      f += "0123456789ABCDEF"[n + 8];
      return f;
    }
  }
  return "";
}

// ============================================================
// BLE Connection Switch
// ============================================================
void ELM327BLESwitch::write_state(bool state) {
  if (this->hub_ != nullptr) {
    this->hub_->set_ble_enabled(state);
  }
  this->publish_state(state);
}

void ELM327BLEHub::setup() {
  ESP_LOGCONFIG(TAG, "ELM327 BLE Hub wird initialisiert...");
}

void ELM327BLEHub::dump_config() {
  ESP_LOGCONFIG(TAG, "ELM327 BLE Hub:");
  ESP_LOGCONFIG(TAG, "  Service UUID: %s", this->service_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  TX Char UUID: %s", this->char_tx_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  RX Char UUID: %s", this->char_rx_uuid_str_.c_str());
  ESP_LOGCONFIG(TAG, "  Abfrageintervall: %u ms", this->request_interval_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u ms", this->request_timeout_);
  ESP_LOGCONFIG(TAG, "  Registrierte PID-Sensoren: %d", (int) this->pid_sensors_.size());
  ESP_LOGCONFIG(TAG, "  Registrierte Raw-PID Text-Sensoren: %d", (int) this->raw_pid_text_sensors_.size());
  if (this->dtc_text_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "  DTC Text Sensor: ja");
  for (auto &entry : this->raw_pid_text_sensors_) {
    ESP_LOGCONFIG(TAG, "  Raw-PID: cmd=%s header=%s rx=%s prefix=%s",
                  entry.command.c_str(), entry.header.c_str(),
                  derive_rx_filter(entry.header).c_str(),
                  entry.expected_prefix.c_str());
  }
}

// ============================================================
// BLE GATTC Event Handler
// ============================================================
void ELM327BLEHub::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                         esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "BLE: Verbunden mit ELM327");
        this->state_ = STATE_CONNECTED;
        if (this->connected_binary_sensor_ != nullptr)
          this->connected_binary_sensor_->publish_state(false);  // noch nicht initialisiert
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "BLE: ELM327 getrennt!");
      this->state_ = STATE_IDLE;
      this->handles_resolved_ = false;
      this->init_step_ = 0;
      this->waiting_for_response_ = false;
      this->response_buffer_.clear();
      this->current_header_.clear();
      this->header_ready_ = false;
      if (this->connected_binary_sensor_ != nullptr)
        this->connected_binary_sensor_->publish_state(false);
      // Switch-State synchronisieren (zeigt den enabled-Status des ble_client)
      if (this->connection_switch_ != nullptr)
        this->connection_switch_->publish_state(this->parent()->enabled);
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Service Discovery abgeschlossen, Handles suchen
      auto *chr_tx = this->parent()->get_characteristic(
          espbt::ESPBTUUID::from_raw(this->service_uuid_str_),
          espbt::ESPBTUUID::from_raw(this->char_tx_uuid_str_));
      auto *chr_rx = this->parent()->get_characteristic(
          espbt::ESPBTUUID::from_raw(this->service_uuid_str_),
          espbt::ESPBTUUID::from_raw(this->char_rx_uuid_str_));

      if (chr_tx == nullptr || chr_rx == nullptr) {
        ESP_LOGE(TAG, "BLE: Characteristics nicht gefunden!");
        break;
      }

      this->char_tx_handle_ = chr_tx->handle;
      this->char_rx_handle_ = chr_rx->handle;
      this->handles_resolved_ = true;

      ESP_LOGI(TAG, "BLE: TX Handle=0x%04X, RX Handle=0x%04X",
               this->char_tx_handle_, this->char_rx_handle_);

      // Notify registrieren fuer RX Characteristic
      auto status = esp_ble_gattc_register_for_notify(
          this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), this->char_rx_handle_);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "BLE: Notify-Registrierung fehlgeschlagen: %d", status);
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      ESP_LOGI(TAG, "BLE: Notify registriert, starte Initialisierung...");
      this->state_ = STATE_INITIALIZING;
      this->init_step_ = 0;
      this->last_init_time_ = 0;
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->char_rx_handle_)
        break;

      // Daten zum Puffer hinzufuegen
      std::string chunk((char *) param->notify.value, param->notify.value_len);
      ESP_LOGV(TAG, "Empfangen (raw): %s", chunk.c_str());
      this->response_buffer_ += chunk;

      // Pruefe ob Antwort komplett (ELM327 sendet '>' als Prompt)
      if (this->response_buffer_.find('>') != std::string::npos) {
        std::string full = this->response_buffer_;
        this->response_buffer_.clear();
        this->process_response(full);
      }
      break;
    }

    default:
      break;
  }
}

// ============================================================
// Main Loop
// ============================================================
void ELM327BLEHub::loop() {
  uint32_t now = millis();

  switch (this->state_) {
    case STATE_INITIALIZING:
      this->run_init_sequence();
      break;

    case STATE_SWITCHING_HEADER: {
      // Sequence UDS en 4 etapes. Le 1003 met ~1,1 s a repondre sur MEB,
      // d'ou le delai plus long avant l'etape finale.
      static const uint32_t step_delay[4] = {500, 500, 500, 1500};
      if (now - this->header_switch_time_ < step_delay[this->header_switch_step_])
        break;

      bool is_broadcast = (this->pending_header_ == "7DF");

      switch (this->header_switch_step_) {
        case 0: {
          // Filtre de reception. Sans lui, l'ELM327 n'ecoute pas l'adresse
          // de reponse du calculateur -> NODATA malgre un ATSH accepte.
          std::string f = is_broadcast ? "" : derive_rx_filter(this->pending_header_);
          std::string cmd = f.empty() ? "ATCRA\r" : ("ATCRA" + f + "\r");
          ESP_LOGD(TAG, "Filtre reception: %s", cmd.c_str());
          this->send_command(cmd);
          this->header_switch_step_ = 1;
          this->header_switch_time_ = now;
          break;
        }

        case 1: {
          std::string cmd = "ATSH" + this->pending_header_ + "\r";
          ESP_LOGD(TAG, "Header-Wechsel: %s", cmd.c_str());
          this->send_command(cmd);
          this->header_switch_step_ = 2;
          this->header_switch_time_ = now;
          break;
        }

        case 2: {
          if (is_broadcast) {
            // Pas de session diagnostic en broadcast : on saute l'etape.
            this->header_switch_step_ = 3;
            this->header_switch_time_ = now - step_delay[3];
          } else {
            // Session diagnostic etendue. Attendue : 5003...
            ESP_LOGD(TAG, "Session diagnostic: 1003");
            this->send_command("1003\r");
            this->header_switch_step_ = 3;
            this->header_switch_time_ = now;
          }
          break;
        }

        default: {
          if (is_broadcast) {
            this->current_header_.clear();
            ESP_LOGD(TAG, "Header zurueckgesetzt auf Broadcast (7DF)");
          } else {
            this->current_header_ = this->pending_header_;
            ESP_LOGD(TAG, "Header pret: %s", this->current_header_.c_str());
          }
          this->pending_header_.clear();
          this->header_switch_step_ = 0;
          this->state_ = STATE_READY;
          this->response_buffer_.clear();
          this->waiting_for_response_ = false;
          // La session expire vite : on enchaine immediatement.
          this->header_ready_ = true;
          this->request_next_pid();
          break;
        }
      }
      break;
    }

    case STATE_READY:
      // Naechste PID-Abfrage senden
      if (!this->waiting_for_response_ && (now - this->last_request_time_ >= this->request_interval_)) {
        this->request_next_pid();
      }
      // Timeout pruefen
      if (this->waiting_for_response_ && (now - this->last_request_time_ >= this->request_timeout_)) {
        ESP_LOGW(TAG, "Timeout nach %ums - keine Antwort erhalten", this->request_timeout_);
        this->waiting_for_response_ = false;
        this->response_buffer_.clear();
      }
      break;

    default:
      break;
  }
}

// ============================================================
// ELM327 Initialisierung
// ============================================================
void ELM327BLEHub::run_init_sequence() {
  uint32_t now = millis();

  struct InitCmd {
    const char *cmd;
    uint32_t delay_after;
    const char *description;
  };

  static const InitCmd init_cmds[] = {
    {"ATZ\r",   2000, "Reset"},
    {"ATE0\r",   500, "Echo aus"},
    {"ATL0\r",   500, "Linefeed aus"},
    {"ATS0\r",   500, "Spaces aus"},
    {"ATH0\r",   500, "Headers aus"},
    {"ATAL\r",   500, "Allow Long Messages (>7 Bytes)"},
    {"ATSTFF\r",  500, "Timeout max (1020ms pro Frame)"},
    {"ATSP7\r", 1000, "Protokoll 7 forciert (CAN 29 bit, 500k)"},
    {"0100\r",  5000, "Protokoll-Erkennung"},
  };

  if (this->init_step_ >= INIT_STEPS_COUNT) {
    // Initialisierung abgeschlossen
    ESP_LOGI(TAG, "ELM327 initialisiert - bereit fuer Abfragen");
    this->state_ = STATE_READY;
    this->current_poll_index_ = 0;
    this->waiting_for_response_ = false;
    this->current_header_.clear();
    this->header_ready_ = false;
    this->update_total_poll_count();
    if (this->connected_binary_sensor_ != nullptr)
      this->connected_binary_sensor_->publish_state(true);
    if (this->connection_switch_ != nullptr)
      this->connection_switch_->publish_state(true);
    return;
  }

  if (this->last_init_time_ == 0 || (now - this->last_init_time_ >= init_cmds[this->init_step_].delay_after)) {
    if (this->last_init_time_ != 0) {
      // Naechster Schritt
      this->init_step_++;
      if (this->init_step_ >= INIT_STEPS_COUNT) {
        // Letzter Delay abgelaufen, fertig
        this->run_init_sequence();
        return;
      }
    }

    ESP_LOGD(TAG, "Init [%d/%d]: %s", this->init_step_ + 1, INIT_STEPS_COUNT,
             init_cmds[this->init_step_].description);
    this->send_command(init_cmds[this->init_step_].cmd);
    this->last_init_time_ = now;
  }
}

// ============================================================
// BLE Write
// ============================================================
void ELM327BLEHub::send_command(const std::string &cmd) {
  if (!this->handles_resolved_) {
    ESP_LOGW(TAG, "Kann nicht senden - BLE Handles nicht aufgeloest");
    return;
  }

  auto status = esp_ble_gattc_write_char(
      this->parent()->get_gattc_if(),
      this->parent()->get_conn_id(),
      this->char_tx_handle_,
      cmd.size(),
      (uint8_t *) cmd.data(),
      ESP_GATT_WRITE_TYPE_RSP,
      ESP_GATT_AUTH_REQ_NONE);

  if (status != ESP_OK) {
    ESP_LOGW(TAG, "BLE Write fehlgeschlagen: %d", status);
  }
}

// ============================================================
// Oeffentlicher Befehl (fuer HA Service-Calls)
// ============================================================
void ELM327BLEHub::send_custom_command(const std::string &cmd) {
  if (this->state_ < STATE_READY) {
    ESP_LOGW(TAG, "ELM327 noch nicht bereit, Befehl ignoriert: %s", cmd.c_str());
    return;
  }
  ESP_LOGI(TAG, "Custom Command: %s", cmd.c_str());
  this->response_buffer_.clear();
  this->send_command(cmd);
}

// ============================================================
// Hilfs-Methoden fuer den Polling-Zyklus
// ============================================================
void ELM327BLEHub::update_total_poll_count() {
  this->total_poll_count_ = this->pid_sensors_.size() + this->raw_pid_text_sensors_.size();
  if (this->dtc_text_sensor_ != nullptr)
    this->total_poll_count_++;
}

std::string ELM327BLEHub::get_header_for_poll_index(int idx) {
  int pid_count = this->pid_sensors_.size();
  int raw_count = this->raw_pid_text_sensors_.size();

  if (idx < pid_count) {
    // Numerische PID-Sensoren haben keinen Header
    return "";
  } else if (idx < pid_count + raw_count) {
    return this->raw_pid_text_sensors_[idx - pid_count].header;
  }
  // DTC: kein Header
  return "";
}

std::string ELM327BLEHub::get_command_for_poll_index(int idx) {
  int pid_count = this->pid_sensors_.size();
  int raw_count = this->raw_pid_text_sensors_.size();

  if (idx < pid_count) {
    return this->pid_sensors_[idx].config.command;
  } else if (idx < pid_count + raw_count) {
    return this->raw_pid_text_sensors_[idx - pid_count].command;
  }
  // DTC
  return "03\r";
}

// ============================================================
// PID-Abfrage-Zyklus
// ============================================================
void ELM327BLEHub::request_next_pid() {
  if (this->total_poll_count_ == 0)
    return;

  int idx = this->current_poll_index_ % this->total_poll_count_;

  std::string needed_header = this->get_header_for_poll_index(idx);

  // La sequence complete (ATCRA + ATSH + 1003) est rejouee AVANT CHAQUE
  // requete a header, meme si le header n'a pas change : la session
  // diagnostic expire en ~5 s cote vehicule.
  if (!needed_header.empty() && !this->header_ready_) {
    ESP_LOGD(TAG, "Sequence UDS pour %s", needed_header.c_str());
    this->pending_header_ = needed_header;
    this->header_switch_step_ = 0;
    this->header_switch_time_ = millis();
    this->state_ = STATE_SWITCHING_HEADER;
    // Index NICHT weiterschalten
    return;
  }
  // Jeton consomme : la prochaine requete a header rejouera la sequence.
  this->header_ready_ = false;

  // Wenn wir von einem Header-Sensor zurueck zu einem ohne Header wechseln,
  // muessen wir den Header auf CAN-Broadcast (7DF) zuruecksetzen
  if (needed_header.empty() && !this->current_header_.empty()) {
    ESP_LOGD(TAG, "Header zuruecksetzen: %s -> 7DF (Broadcast)", this->current_header_.c_str());
    this->pending_header_ = "7DF";
    this->header_switch_step_ = 0;
    this->header_switch_time_ = millis();
    this->state_ = STATE_SWITCHING_HEADER;
    return;
  }

  std::string cmd = this->get_command_for_poll_index(idx);
  int pid_count = this->pid_sensors_.size();
  int raw_count = this->raw_pid_text_sensors_.size();

  // Request-Log: Befehl + Header auf INFO-Level fuer Ping-Pong Sichtbarkeit
  if (idx < pid_count) {
    ESP_LOGI(TAG, ">> [%d/%d] %s", idx + 1, this->total_poll_count_, cmd.c_str());
  } else if (idx < pid_count + raw_count) {
    std::string hdr = this->raw_pid_text_sensors_[idx - pid_count].header;
    if (!hdr.empty()) {
      ESP_LOGI(TAG, ">> [%d/%d] %s (ECU %s)", idx + 1, this->total_poll_count_, cmd.c_str(), hdr.c_str());
    } else {
      ESP_LOGI(TAG, ">> [%d/%d] %s", idx + 1, this->total_poll_count_, cmd.c_str());
    }
  } else {
    ESP_LOGI(TAG, ">> [%d/%d] DTC Abfrage", idx + 1, this->total_poll_count_);
  }

  this->response_buffer_.clear();
  this->waiting_for_response_ = true;
  this->last_request_time_ = millis();
  this->send_command(cmd);
  this->current_poll_index_ = (idx + 1) % this->total_poll_count_;
}

// ============================================================
// Antwort-Verarbeitung
// ============================================================
void ELM327BLEHub::process_response(const std::string &response) {
  this->waiting_for_response_ = false;

  // Bereinigen: Whitespace und ELM327-Prompt entfernen
  std::string clean;
  for (char c : response) {
    if (c != ' ' && c != '\r' && c != '\n' && c != '>') {
      clean += c;
    }
  }

  // ISO-TP Multi-Frame Segment-Nummern entfernen
  // Bei langen Antworten fuegt der ELM327 Zeilen-Nummern ein: 0: 1: 2: ... F:
  // z.B. "620101EFFBE71:ED950000..." -> "620101EFFBE7ED950000..."
  {
    std::string stripped;
    stripped.reserve(clean.size());
    for (size_t i = 0; i < clean.size(); i++) {
      // Pruefe auf Muster: einzelnes Hex-Zeichen gefolgt von ':'
      if (i + 1 < clean.size() && clean[i + 1] == ':') {
        char c = clean[i];
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
          // Segment-Nummer + ':' ueberspringen
          i += 1;  // das ':' wird uebersprungen, die for-Schleife macht i++
          continue;
        }
      }
      stripped += clean[i];
    }
    clean = stripped;
  }

  ESP_LOGI(TAG, "<< %s", clean.c_str());

  // Debug: Raw Text Sensor (alle Antworten)
  if (this->raw_text_sensor_ != nullptr) {
    this->raw_text_sensor_->publish_state(clean);
  }

  // Antworten der UDS-Sequenz (OK / 5003...) ignorieren
  if (this->state_ == STATE_SWITCHING_HEADER) {
    return;
  }

  // Fehler ignorieren
  if (clean.find("NODATA") != std::string::npos ||
      clean.find("ERROR") != std::string::npos ||
      clean.find("UNABLE") != std::string::npos ||
      clean.find("STOPPED") != std::string::npos) {
    ESP_LOGW(TAG, "Fehler/Keine Daten: %s", clean.c_str());
    return;
  }

  // AT-Antworten ignorieren (z.B. "OK", "ATSH7E4" Echo)
  if (clean == "OK" || clean.find("AT") == 0) {
    return;
  }

  // Raw-PID Text-Sensoren pruefen (vor dem spezifischen Parsing)
  // Damit werden alle Responses, die zu einem raw_pid Sensor passen,
  // als Hex-String weitergegeben
  bool raw_matched = this->dispatch_raw_pid_response(clean);
  if (raw_matched)
    return;

  // DTC-Antwort (Mode 03, beginnt mit "43")
  if (clean.find("43") != std::string::npos) {
    this->parse_dtc_response(clean);
    return;
  }

  // Batteriespannung (ATRV, z.B. "12.4V")
  if (clean.find("V") != std::string::npos && clean.find("41") == std::string::npos) {
    this->parse_voltage_response(clean);
    return;
  }

  // OBD2 Mode 01 Antwort (beginnt mit "41")
  if (clean.find("41") != std::string::npos) {
    this->parse_obd2_response(clean);
    return;
  }

  // Mode 22 Antwort ohne Sensor
  if (clean.find("62") != std::string::npos) {
    ESP_LOGW(TAG, "Mode 22 Response ohne passenden Sensor: %s", clean.c_str());
    return;
  }
}

// ============================================================
// Raw-PID Text-Sensor Dispatching
// ============================================================
bool ELM327BLEHub::dispatch_raw_pid_response(const std::string &clean) {
  bool matched = false;
  for (auto &entry : this->raw_pid_text_sensors_) {
    if (entry.expected_prefix.empty())
      continue;

    // Pruefen ob die Response den erwarteten Prefix enthaelt
    size_t pos = clean.find(entry.expected_prefix);
    if (pos != std::string::npos) {
      std::string data = clean.substr(pos);
      // Schutz gegen abgeschnittene Responses : au moins 1 octet utile.
      // ATTENTION : le seuil d'origine (prefix + 8 caracteres) rejetait
      // silencieusement 62028CEF et 62295A0086FB. Ne pas le remonter.
      size_t min_len = entry.expected_prefix.length() + 2;
      if (data.length() < min_len) {
        ESP_LOGW(TAG, "Response zu kurz (%d Zeichen, erwartet >=%d): %s",
                 (int) data.length(), (int) min_len, data.c_str());
        matched = true;  // trotzdem als matched zaehlen
        continue;        // aber NICHT publizieren
      }
      entry.sensor->publish_state(data);
      ESP_LOGD(TAG, "Raw-PID Match [%s] -> %s", entry.expected_prefix.c_str(), data.c_str());
      matched = true;
    }
  }
  return matched;
}

// ============================================================
// OBD2 PID Parsing (Mode 01)
// ============================================================
void ELM327BLEHub::parse_obd2_response(const std::string &clean) {
  size_t pos = clean.find("41");
  if (pos == std::string::npos) return;

  std::string data = clean.substr(pos);
  if (data.length() < 6) return;

  auto hex_byte = [](const std::string &s, int off) -> int {
    if (off + 2 > (int) s.length()) return -1;
    return (int) strtol(s.substr(off, 2).c_str(), nullptr, 16);
  };

  int pid = hex_byte(data, 2);
  int a = hex_byte(data, 4);
  int b = (data.length() >= 8) ? hex_byte(data, 6) : 0;

  if (pid < 0 || a < 0) return;

  sensor::Sensor *sensor = this->find_sensor_for_pid(pid);
  if (sensor == nullptr) {
    ESP_LOGD(TAG, "Kein Sensor fuer PID 0x%02X registriert", pid);
    return;
  }

  float value = 0;
  switch (pid) {
    case 0x04:  // Motorlast: A*100/255
      value = (a * 100.0f) / 255.0f;
      break;
    case 0x05:  // Kuehlmitteltemperatur: A - 40
      value = a - 40.0f;
      break;
    case 0x0B:  // MAP: A
      value = a;
      break;
    case 0x0C:  // Drehzahl: ((A*256)+B)/4
      value = ((a * 256) + b) / 4.0f;
      break;
    case 0x0D:  // Geschwindigkeit: A
      value = a;
      break;
    case 0x0F:  // Ansauglufttemperatur: A - 40
      value = a - 40.0f;
      break;
    case 0x10:  // MAF: ((A*256)+B)/100
      value = ((a * 256) + b) / 100.0f;
      break;
    case 0x11:  // Drosselklappe: A*100/255
      value = (a * 100.0f) / 255.0f;
      break;
    case 0x1F:  // Motorlaufzeit: (A*256)+B
      value = (a * 256) + b;
      break;
    case 0x2E:  // AGR: A*100/255
      value = (a * 100.0f) / 255.0f;
      break;
    case 0x2F:  // Kraftstoffstand: A*100/255
      value = (a * 100.0f) / 255.0f;
      break;
    case 0x33:  // Barometrischer Druck: A
      value = a;
      break;
    case 0x42:  // ECU Spannung: ((A*256)+B)/1000
      value = ((a * 256) + b) / 1000.0f;
      break;
    case 0x46:  // Umgebungstemperatur: A - 40
      value = a - 40.0f;
      break;
    case 0x5C:  // Oeltemperatur: A - 40
      value = a - 40.0f;
      break;
    case 0x5E:  // Kraftstoffverbrauch: ((A*256)+B)/20
      value = ((a * 256) + b) / 20.0f;
      break;
    default:
      // Generische Formel: einfach A zurueckgeben
      value = a;
      ESP_LOGD(TAG, "PID 0x%02X: generisch A=%d", pid, a);
      break;
  }

  sensor->publish_state(value);
  ESP_LOGD(TAG, "PID 0x%02X = %.2f", pid, value);

  // Motor-Lauf-Status aktualisieren (basierend auf RPM)
  if (pid == 0x0C && this->engine_running_binary_sensor_ != nullptr) {
    this->engine_running_binary_sensor_->publish_state(value > 0);
  }
}

// ============================================================
// DTC Parsing
// ============================================================
void ELM327BLEHub::parse_dtc_response(const std::string &clean) {
  if (this->dtc_text_sensor_ == nullptr) return;

  size_t dpos = clean.find("43");
  if (dpos == std::string::npos) return;

  std::string dtc_data = clean.substr(dpos);
  std::string dtc_list;
  int dtc_count = 0;

  // DTCs starten ab Position 2 (nach "43"), je 4 Hex-Zeichen = 2 Bytes
  for (size_t i = 2; i + 3 < dtc_data.length(); i += 4) {
    std::string dtc_raw = dtc_data.substr(i, 4);
    if (dtc_raw == "0000") continue;

    std::string dtc_code = this->decode_dtc(dtc_raw);
    if (!dtc_list.empty()) dtc_list += ", ";
    dtc_list += dtc_code;
    dtc_count++;
  }

  if (dtc_count == 0) {
    this->dtc_text_sensor_->publish_state("Keine Fehler");
  } else {
    this->dtc_text_sensor_->publish_state(dtc_list);
  }
  ESP_LOGD(TAG, "DTCs (%d): %s", dtc_count, dtc_list.c_str());
}

std::string ELM327BLEHub::decode_dtc(const std::string &raw) {
  if (raw.length() < 4) return "????";

  char first = raw[0];
  char prefix;
  switch (first) {
    case '0': case '1': case '2': case '3': prefix = 'P'; break;
    case '4': case '5': prefix = 'C'; break;
    case '6': case '7': prefix = 'B'; break;
    case '8': case '9': case 'A': case 'B': prefix = 'U'; break;
    default: prefix = 'P'; break;
  }

  char second = '0' + (((first >= 'A' ? first - 'A' + 10 : first - '0') % 4));

  std::string result;
  result += prefix;
  result += second;
  result += raw.substr(1, 3);
  return result;
}

// ============================================================
// Batteriespannung (ATRV)
// ============================================================
void ELM327BLEHub::parse_voltage_response(const std::string &clean) {
  sensor::Sensor *sensor = this->find_at_sensor("ATRV\r");
  if (sensor == nullptr) return;

  std::string volt_str;
  for (char c : clean) {
    if ((c >= '0' && c <= '9') || c == '.') {
      volt_str += c;
    }
  }

  if (!volt_str.empty()) {
    float voltage = atof(volt_str.c_str());
    if (voltage > 0 && voltage < 20) {
      sensor->publish_state(voltage);
      ESP_LOGD(TAG, "Batterie: %.1f V", voltage);
    }
  }
}

// ============================================================
// Sensor-Registrierung
// ============================================================
void ELM327BLEHub::register_pid_sensor(sensor::Sensor *sensor, uint8_t mode, uint16_t pid) {
  PIDSensorEntry entry;
  entry.sensor = sensor;
  entry.config.mode = mode;
  entry.config.pid = pid;
  entry.config.is_at_command = false;
  // Befehl generieren: z.B. mode=0x01 pid=0x05 -> "0105\r"
  // Fuer Mode 22 mit erweiterten PIDs: mode=0x22 pid=0x0101 -> "220101\r"
  char cmd[16];
  if (pid <= 0xFF) {
    snprintf(cmd, sizeof(cmd), "%02X%02X\r", mode, pid);
  } else {
    snprintf(cmd, sizeof(cmd), "%02X%04X\r", mode, pid);
  }
  entry.config.command = cmd;
  this->pid_sensors_.push_back(entry);
  ESP_LOGD(TAG, "PID Sensor registriert: Mode 0x%02X PID 0x%04X -> %s", mode, pid, cmd);
}

void ELM327BLEHub::register_at_sensor(sensor::Sensor *sensor, const std::string &command) {
  PIDSensorEntry entry;
  entry.sensor = sensor;
  entry.config.mode = 0;
  entry.config.pid = 0;
  entry.config.is_at_command = true;
  entry.config.command = command;
  this->pid_sensors_.push_back(entry);
  ESP_LOGD(TAG, "AT Sensor registriert: %s", command.c_str());
}

void ELM327BLEHub::register_dtc_text_sensor(text_sensor::TextSensor *sensor) {
  this->dtc_text_sensor_ = sensor;
}

void ELM327BLEHub::register_raw_text_sensor(text_sensor::TextSensor *sensor) {
  this->raw_text_sensor_ = sensor;
}

void ELM327BLEHub::register_raw_pid_text_sensor(text_sensor::TextSensor *sensor, uint8_t mode,
                                                  uint16_t pid, const std::string &header,
                                                  const std::string &command) {
  RawPIDTextSensorEntry entry;
  entry.sensor = sensor;
  entry.mode = mode;
  entry.pid = pid;
  entry.header = header;

  if (!command.empty()) {
    // Beliebiger Befehl (direkt uebergeben)
    entry.command = command;
    // Wenn command mit \r endet, das fuer den Prefix entfernen
    std::string cmd_clean = command;
    if (!cmd_clean.empty() && cmd_clean.back() == '\r')
      cmd_clean.pop_back();
    // Erwarteten Response-Prefix berechnen:
    // Mode XX -> Response-Prefix = (mode + 0x40) + PID
    // z.B. "2201019" -> Response beginnt mit "6201019"
    // Fuer beliebige Commands versuchen wir den Prefix zu berechnen
    if (cmd_clean.length() >= 2) {
      int cmd_mode = (int) strtol(cmd_clean.substr(0, 2).c_str(), nullptr, 16);
      if (cmd_mode > 0 && cmd_mode < 0x40) {
        char prefix[16];
        snprintf(prefix, sizeof(prefix), "%02X%s", cmd_mode + 0x40, cmd_clean.substr(2).c_str());
        entry.expected_prefix = prefix;
      }
    }
  } else {
    // Command aus mode + pid generieren
    char cmd[16];
    if (pid <= 0xFF) {
      snprintf(cmd, sizeof(cmd), "%02X%02X\r", mode, pid);
    } else {
      snprintf(cmd, sizeof(cmd), "%02X%04X\r", mode, pid);
    }
    entry.command = cmd;

    // Erwarteten Response-Prefix berechnen
    char prefix[16];
    if (pid <= 0xFF) {
      snprintf(prefix, sizeof(prefix), "%02X%02X", mode + 0x40, pid);
    } else {
      snprintf(prefix, sizeof(prefix), "%02X%04X", mode + 0x40, pid);
    }
    entry.expected_prefix = prefix;
  }

  this->raw_pid_text_sensors_.push_back(entry);
  ESP_LOGD(TAG, "Raw-PID Text-Sensor registriert: cmd=%s header=%s rx=%s prefix=%s",
           entry.command.c_str(), entry.header.c_str(),
           derive_rx_filter(entry.header).c_str(), entry.expected_prefix.c_str());
}

void ELM327BLEHub::register_connected_binary_sensor(binary_sensor::BinarySensor *sensor) {
  this->connected_binary_sensor_ = sensor;
}

void ELM327BLEHub::register_engine_running_binary_sensor(binary_sensor::BinarySensor *sensor) {
  this->engine_running_binary_sensor_ = sensor;
}

void ELM327BLEHub::register_connection_switch(switch_::Switch *sw) {
  this->connection_switch_ = sw;
}

// ============================================================
// BLE-Verbindung steuern (Switch)
// ============================================================
void ELM327BLEHub::set_ble_enabled(bool enabled) {
  ESP_LOGI(TAG, "BLE-Verbindung %s", enabled ? "aktiviert" : "deaktiviert");
  this->parent()->set_enabled(enabled);
  if (this->connection_switch_ != nullptr) {
    this->connection_switch_->publish_state(enabled);
  }
  if (!enabled) {
    // State zuruecksetzen
    this->state_ = STATE_IDLE;
    this->handles_resolved_ = false;
    this->init_step_ = 0;
    this->waiting_for_response_ = false;
    this->response_buffer_.clear();
    this->current_header_.clear();
    this->header_ready_ = false;
    if (this->connected_binary_sensor_ != nullptr)
      this->connected_binary_sensor_->publish_state(false);
  }
}

// ============================================================
// Sensor Lookup
// ============================================================
sensor::Sensor *ELM327BLEHub::find_sensor_for_pid(uint8_t pid) {
  for (auto &entry : this->pid_sensors_) {
    if (!entry.config.is_at_command && entry.config.pid == pid) {
      return entry.sensor;
    }
  }
  return nullptr;
}

sensor::Sensor *ELM327BLEHub::find_at_sensor(const std::string &command) {
  for (auto &entry : this->pid_sensors_) {
    if (entry.config.is_at_command && entry.config.command == command) {
      return entry.sensor;
    }
  }
  return nullptr;
}

}  // namespace elm327_ble
}  // namespace esphome
