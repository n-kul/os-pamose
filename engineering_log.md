Date:
24-05-2026

Objective:
flea market reconnaissance

Acquired:
FM radio x2
dB Spectrum+

Results:
Radio 1 repaired

Issues:
Spectrum power path failure
Internal contamination
No display available

Next steps:
cleaning
power tracing
output identification
keyboard usability verification


Date:
25-05-2026

Objective:
Figure out whether the ₹100 Spectrum is dead, alive, cursed or planning something.

Work performed:

Cleaned the inside.

Expected:
dust

Actual:
small ecosystem

PCB surprisingly okay.

Identified:

Z80A

M27256 EPROM

ULA6C001E7

UHF EU36 modulator

Spent unreasonable amount of time thinking regulator layout was wrong.

Turns out I wired barrel polarity wrong.

Very humbling experience.

Checked power again.

RESET goes to ~4V.

CPU warm.

EPROM warm.

ULA warm.

Machine refuses to behave like a corpse.

RF noise changes near modulator.

Meaning:
thing is doing something.

Video still missing.

Speaker randomly buzzed at one point.

Possibly life.
Possibly confusion.

Removed PCB safely.

Keyboard investigation:

Good news:

membrane intact

Bad news:

ribbon/tail damaged

Current state:

computer maybe alive

video unknown

keyboard injured

shell excellent

Started getting dangerous ideas.

New concept:

Use Spectrum shell for PAMOSE.

Headless operation.

No Galaxy Ace.

Connect from whatever device exists.

Planned layout:

Spectrum keyboard
→ Pro Micro

Pro Micro
→ UART

UART
→ ESP32

ESP32
→ WiFi interface

Future power:

4×18650

BMS

USB cable for power + upload

Issues:

No display

No replacement ribbon

Keyboard repair pain incoming

Next steps:

Check CRT

Try keyboard repair

Avoid destroying membrane

Do not commit historical crimes against PCB

Notes:

Went to flea market for display.

Returned with:

2 radios

1 retro computer

possible cyberdeck enclosure