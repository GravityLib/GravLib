use vexide::devices::math::Point2;
use vexide::prelude::*;
use vexide::devices::{display::*};

pub fn gravlib_logo(display: &mut Display) {
    // common colours & x‑position
    let yellow = Rgb::new(255, 255, 0);
    let black  = Rgb::new(0, 0, 0);
    let x: i16  = 10;

    // 1) Top banner, art rows, and bottom banner
    let mut y: i16 = 15;
    let banner_rows: &[(&str, FontSize)] = &[
        ("################################################", FontSize::SMALL),
        ("", FontSize::EXTRA_SMALL),
        (" $$$$$$\\                               $$\\       $$\\ $$\\       ", FontSize::EXTRA_SMALL),
        ("$$  __$$\\                              $$ |      \\__|$$ |      ", FontSize::EXTRA_SMALL),
        ("$$ /  \\__| $$$$$$\\  $$$$$$\\ $$\\    $$\\ $$ |      $$\\ $$$$$$$\\  ", FontSize::EXTRA_SMALL),
        ("$$ |$$$$\\ $$  __$$\\ \\____$$\\\\$$\\  $$  |$$ |      $$ |$$  __$$\\ ", FontSize::EXTRA_SMALL),
        ("$$ |\\_$$ |$$ |  \\__|$$$$$$$ |\\$$\\$$  / $$ |      $$ |$$ |  $$ |", FontSize::EXTRA_SMALL),
        ("\\$$$$$$  |$$ |     \\$$$$$$$ |  \\$  /   $$$$$$$$\\ $$ |$$$$$$$  |", FontSize::EXTRA_SMALL),
        (" \\______/ \\__|      \\_______|   \\_/    \\________|\\__|\\_______/ ", FontSize::EXTRA_SMALL),
        ("", FontSize::EXTRA_SMALL),
        ("################################################", FontSize::SMALL),
    ];

    for &(line, size) in banner_rows {
        let font = Font::new(size, FontFamily::Monospace);
        display.draw_text(
            &Text::new(line, font, Point2::<i16>::from([x, y])),
            yellow,
            Some(black),
        );
        y += match size {
            FontSize::SMALL       => 15,
            FontSize::EXTRA_SMALL => 10,
            _                     => 12,
        };
    }

    // 2) Description block
    let desc_base_y = y + 15;
    let desc_spacing = 15;
    let description = [
        "",
        "   GravLib v0.1 (Pre-release Version)",
        "   By Alex Cai (LycoKodo) & GravLib Contributors",
        "   \"All Hail Nijika Ijichi\"",
    ];

    for (i, &line) in description.iter().enumerate() {
        let yy = desc_base_y + (i as i16 * desc_spacing);
        display.draw_text(
            &Text::new(
                line,
                Font::new(FontSize::SMALL, FontFamily::Monospace),
                Point2::<i16>::from([x, yy]),
            ),
            yellow,
            Some(black),
        );
    }
}
