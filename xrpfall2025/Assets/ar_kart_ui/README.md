# AR Kart Racing: UI assets

Put this whole folder in your Godot project at `res://Assets/ar_kart_ui/`. The theme file expects the font at `res://Assets/ar_kart_ui/fonts/Rubik.ttf`. If you put the folder somewhere else, change that one path at the top of `godot/ar_kart_theme.tres`.

## What's inside

| Folder | Contents |
|---|---|
| `logos/` | AR Kart Racing crossed-flags logo (SVG + PNG @1x/2x/4x). XRP logo (regular + white-letter `on-dark` version). Cornell Cup Robotics logo. All are vectors. |
| `buttons/` | Blank pill buttons, 320×92, in red / blue / white / black: `normal`, `hover`, `pressed`, plus `disabled`. SVG + PNG @1x/@2x. No text: Godot draws the label. |
| `icons/` | Round icon buttons (pause, settings, back, restart, sound, home) and the bare glyphs in white and dark, including fast / slow / inverted / warning. |
| `items/` | Item badges for Fast (red), Slow (blue), Inverted (black), plus an empty slot. |
| `hud/` | Place badge, speed-bar segments 1–6 (on) + off, dark and light panels, a tileable checkered strip. |
| `screens/` | 2560×1440 reference renders of the title, kart select, race HUD and results screens. |
| `fonts/` | Rubik (variable font) and its OFL license. Free for games, including commercial ones. |
| `godot/` | `ar_kart_theme.tres` (Godot 4 theme) and `streak_background.gd` (animated background). |

## Using the theme (recommended for buttons and panels)

The theme rebuilds the buttons with `StyleBoxFlat`, so they stay sharp at any resolution and stretch to any width.

1. Select your root UI Control. In the Inspector, set **Theme** to `godot/ar_kart_theme.tres`.
2. Every `Button` is now the red pill with the thick bottom edge, plus hover, pressed, disabled and focus states.
3. For the other colors, set a Button's **Theme Type Variation** to `BlueButton`, `WhiteButton` or `BlackButton`.
4. `PanelContainer` gets the dark rounded panel. Use the `LightPanel` variation for the white card, such as the lap times card. Use the `DarkLabel` variation for text on white.

The theme leaves out the glossy highlight stripe. If you want it, add a `TextureRect` or `Panel` child near the top of the button, about 18% of its height, in white at 30% opacity. Or use the PNG/SVG buttons in `buttons/` with a `TextureButton`.

## Animated background

1. Add a `Control` as the first child of your menu scene and set its layout to **Full Rect**.
2. Attach `godot/streak_background.gd`.
3. Adjust `streak_count`, `min_speed`, `max_speed` and `angle_degrees` in the Inspector.

## Colors

| Use | Hex |
|---|---|
| XRP red / light / edge | `#D0112B` / `#F0334A` / `#8E0A1C` |
| XRP blue / light / edge | `#0070FF` / `#4D9BFF` / `#0052C2` |
| Navy background | `#161C28` |
| Darkest | `#0B0F17` |
| Grey (lines, edges) | `#C3CCDA`, `#9AA6B8`, `#5B6578` |
| Accent yellow (RACING) | `#F7C531` |

## Notes

- The XRP and Cornell Cup logos were traced from PNGs. If you get the official vector files, swap them in.
- The XRP logo is XRP's trademark and the Cornell Cup logo is Cornell Cup Robotics'. Use them in line with their guidelines.
