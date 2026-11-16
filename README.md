# Cheo

Cheo is a desk buddy robot with screen eyes. He runs **CDBos** (Cheo Desk Buddy Operating System) on an ESP32-C6 or ESP32-S3, talks through Groq, and updates himself from this repo.

## What's here

| Path | What it is |
|---|---|
| `cdbos/cdbos.ino` | The firmware (one sketch for both boards) |
| `version.txt` | The newest version number. Cheo compares it to his own |
| `cheo.bin` | The compiled firmware for the **ESP32-C6** |
| `cheo-s3.bin` | The compiled firmware for the **ESP32-S3** |
| `docs/` | Biped concept picture and walking-simulation report |
| `leg/` | Printable biped leg (STL files), joint limits, and the design scripts |

## First install (USB)

1. Open `cdbos/cdbos.ino` in Arduino IDE (ESP32 core 3.x).
2. Install the libraries: Adafruit GFX, Adafruit ST7735 and ST7789, ArduinoJson 7, NimBLE-Arduino 2.x.
3. Fill in the five `SETUP_` lines (Wi-Fi, hotspot, Groq key) **for this one upload only**.
4. Pick the board settings:

| | ESP32-C6 | ESP32-S3 (16MB) |
|---|---|---|
| Board | ESP32C6 Dev Module | ESP32S3 Dev Module |
| Flash Size | 4MB | 16MB |
| Partition Scheme | Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS) | 16M Flash (3MB APP/9.9MB FATFS) |
| PSRAM | — | OPI PSRAM |

5. Upload. Cheo saves the Wi-Fi and key inside himself.
6. **Blank the `SETUP_` lines again before you commit anything.** Never commit passwords or keys.

After that, change Wi-Fi networks and the Groq key at `cheo.local/settings`. If Cheo can't find any saved Wi-Fi, he makes his own network called **Cheo-Setup** (open `192.168.4.1`).

## Releasing an update (OTA)

1. Raise `FW_VERSION` in `cdbos/cdbos.ino`.
2. Check that the `SETUP_` lines are `""`.
3. For each board, select it in Tools, then **Sketch → Export Compiled Binary**. Rename the main `.bin` (not the bootloader/partitions/merged ones):
   - ESP32-C6 → `cheo.bin`
   - ESP32-S3 → `cheo-s3.bin`
4. Commit the `.bin` files to the repo root.
5. **Last**, set `version.txt` to the new number. Only bump it after the matching `.bin` files are uploaded, or Cheo will install the old firmware over and over.

Cheo checks 1 minute after he starts and every 6 hours, or when you say "check for updates". GitHub can take about 5 minutes to show new files.

## Wiring

| Part | ESP32-C6 | ESP32-S3 |
|---|---|---|
| Screen SCK / SDA / RES / DC / CS | 6 / 7 / 20 / 19 / 18 | 10 / 11 / 12 / 13 / 14 |
| Screen VCC + BLK | 3V3 | 3V3 |
| Amp BCLK / LRC / DIN | 21 / 22 / 23 | 4 / 5 / 6 |
| Amp VIN | 5V | 5V |
| Servo signal | 2 | 7 |
| Ultrasonic TRIG / ECHO (ECHO through a divider) | 3 / 4 | 15 / 16 |
| Microphone INMP441 SCK / WS / SD | — | 1 / 2 / 42 |
| Mic VDD, GND, L/R | — | 3V3, GND, GND |
| Button | BOOT (9) | BOOT (0) |
| Screen rotation | 3 | 1 |

## Cheple link API

On the same Wi-Fi, the Cheple phone (or anything else) can control Cheo:

| Request | Does |
|---|---|
| `cheo.local/api/status` | JSON: mood, pet stats, next reminder, weather, last reply |
| `cheo.local/api/say?t=hello&m=happy` | Says something, with an optional face |
| `cheo.local/api/ask?q=...` | Asks the AI; read the answer from `/last` |
| `cheo.local/api/timer?s=300&l=pasta` | Starts a timer |
| `cheo.local/api/face?m=wink` | Shows a face |
| `cheo.local/api/show?s=clock` | eyes, clock, weather, pet, count |
| `cheo.local/api/feed?f=cheese` | Feeds him |

Cheo also broadcasts events on UDP port **4211** as `CHEO|type|text` (types: `alarm`, `timer`, `reminder`, `notif`, `call`, `levelup`, `rain`).

## Talking to Cheo (v12, ESP32-S3)

**Hold his BOOT button and talk, then let go.** Or hold the 🎤 button on his web page. He records up to 10 seconds, Groq Whisper turns it into words, and he answers out loud.
Study mode moved off the button hold: say "start study mode" or use the web page.

## Biped leg (leg/)

- `leg/stl/`: `L_` = left leg, `R_` = right leg (mirror image). `shoe_3.0mm.stl` x5 per leg (2.5 and 3.5 mm versions to fine-tune the fit).
- `leg/JOINT_LIMITS.md`: how far each joint can move before parts hit.
- `leg/leg_limits.h`: the same limits for the firmware.
