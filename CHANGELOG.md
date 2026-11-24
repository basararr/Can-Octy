# Değişiklik Geçmişi (Changelog)

Projedeki önemli güncellemelerin, hata düzeltmelerinin ve donanım geçişlerinin özeti.

## 2026-11-24 — Derin Donanım, Protokol ve Sıfır-Heap Güçlendirmeleri

- **Gerçek Wi-Fi AP ve CYW43439 Kapatma (`BUS_SLEEP`):** `arduino-pico` çekirdeğinde `WiFi.softAPdisconnect()` çağrısının `WIFI_AP` modunda AP arayüzünü kapatmaması nedeniyle uyku geçişi `LittleFS.end()`, `WiFi.disconnectAP()`, `WiFi.mode(WIFI_OFF)` ve `cyw43_arch_deinit()` sıralamasıyla güncellendi; uyku modunda kablosuz çip tamamen enerjisiz bırakıldı.
- **MCP2515 Takılı `TXREQ` Temizliği:** Araç kontağı kapandığında veya ACK veren düğüm olmadığında `TXB0..TXB2` içinde asılı kalan çerçevelerin (`0x54` durum maskesi) `Listen-Only` geçişini kilitlememesi için SPI donanım reseti (`can->reset()`) ile kurtarma eklendi.
- **Tam Sıfır-Heap WebSocket Yayını:** `WebSockets` kütüphanesinin RP2040/RP2350 üzerinde her gönderimde arka planda yaptığı `malloc`/`free` çağrılarını devre dışı bırakmak için telemetri ve log tamponlarında `WEBSOCKETS_MAX_HEADER_SIZE` (14 byte) başlık alanı ayrılıp `headerToPayload = true` moduna geçildi.
- **OTA Dosya Sistemi Ön-Hazırlığı:** `ArduinoOTA` başlamadan önce `onStart()` dışında gecikme yaşanmaması için `setup()` içinde `LittleFS.begin()` ön-bağlaması eklendi.
- **İstemci Etkileşimli Uyku Sayacı:** Masaüstü / hareketsizlik uykusu cihazın toplam çalışma süresi yerine son istemci etkileşiminden (`g_lastClientActivityMs`: HTTP, WebSocket ve OTA) itibaren 5 dakika sayacak şekilde düzeltildi; 5. dakikadan sonra sayfa yenilemede uykuya geçme sorunu giderildi.
- **ISO-TP / UDS ve Log Tamponu:** First Frame için 8-byte DLC kontrolü, Flow Control (`0x30`) gönderim hata kontrolü ve ardışık `NRC 0x78` (ResponsePending) için üst sınır (`10`) eklendi. Log halka tamponu `32` satırdan `64` satıra çıkarıldı.
- **MQB Modül Adresleri ve Arayüz Senkronizasyonu:** Tarayıcıdaki `CentElec (09)` adresi gerçek MQB BCM adresi olan `0x70E -> 0x778` ile değiştirildi, `DrvDoor (42)` (`0x74A -> 0x7B4`) ve hızlı seçim butonları eklendi. C++ ve JS hex doğrulaması eşitlendi; menü dış tıklama, ayar modalı `İptal` butonu ve sekme dönüşünde anlık veri yenileme eklendi.

## 2026-11-21 — Canlı Simülatör, Dokümantasyon ve Çift Çekirdek Modernizasyonu

- **GitHub Pages Canlı Simülatörü (`docs/index.html`):** Arabaya bağlanmadan tarayıcıda çalışan interaktif MFD ekranı, sürüş ve aktif DPF rejenerasyon senaryoları, canlı CAN-Bus çerçeve akışı ve 24 DID ham hex çözücü eklendi.
- **Donanım ve Watchdog İyileştirmeleri:** MCP2515 donanım modu önbelleğe alınarak gereksiz SPI `setNormalMode()` çağrıları kaldırıldı; Core 1 heartbeat kontrolüne bağlı donanım watchdog koruması ve arayüzde `> 0x7FF` otomatik RX uyarısı eklendi.
- **`vag_pico_custom_query` Modernizasyonu:**
  - SPI pinlerinin MCP2515 kurulumundan önce başlatılması düzeltildi.
  - `std::queue<String>` ve dinamik JSON birleştirmeleri kaldırılıp `auto_init_mutex` korumalı sabit tamponlara geçildi.
  - `0x114E` (Soot Meas) ve `0x14F5` (DPF dP) için işaretli `int16_t` çözümlemesi ve büyük harf JSON anahtar uyumu düzeltildi.
  - VAG MQB `+0x08` / `+0x6A` yanıt kuralı, çok çerçeveli ISO-TP okuma, pasif `Listen-Only` uyku/uyanma akışı ve 60 FPS çift yönlü beyaz turbo/vakum barı kuruldu.
- **Yapılandırma ve README:** Wi-Fi bilgileri tüm Pico kodlarında `.gitignore` altındaki `secrets.h` dosyasına taşındı (`secrets.h.example` eklendi); arayüz ekran görüntüleri ve 24 EA288 DID tablosu `README.md`'ye işlendi.

## Önceki Aşamalar (Arşiv ve Gelişim Süreci)

- **Pico 2 W Geçişi (`vag_pico_custom_query`, `vag_pico_all_in_one`, `vag_pico_latency_test`):** Çift çekirdekli RP2350 mimarisine geçiş (Core 1 CAN-Bus, Core 0 Wi-Fi AP + WebSocket + OTA), PIO-USB üzerinden flash belleğe CSV kayıt denemesi ve ardından ham UDS **Özel Sorgu** sekmesinin eklenmesi.
- **Arduino Nano + OLED ISO-TP Dönemi (`vag_mfd_display v1.3`, `vag_did_validator v6.1`, `vag_module_scanner v1.1`, `archive/nano-isotp`):** ISO-TP akış kontrolüne geçiş, 44 modüllük VAG MQB tarayıcı ve EA288 1.6 TDI (`0x7E0`) üzerindeki 24 DID adresi ile ölçekleme formüllerinin araç üstünde deneyerek çıkarılması.
- **İlk Prototipler (`archive/nano-early`, `archive/pico-early`):** İlk Arduino Nano + SSD1306 OLED göstergeleri (`v1.0`–`v3`), masaüstü test simülatörleri ve ilk SD kartlı Pico 2 W denemeleri.
