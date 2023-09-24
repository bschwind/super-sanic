use crate::EXTERNAL_CRYSTAL_FREQUENCY_HZ;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::{
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC},
    Watchdog,
};

const XOSC_KHZ: u32 = 12_000;
const PICO_PLL_VCO_MIN_FREQ_KHZ: u32 = 750_000; // 750 MHz
const PICO_PLL_VCO_MAX_FREQ_KHZ: u32 = 1_600_000; // 1600 MHz

pub fn set_system_clock_exact(
    system_frequency: HertzU32,
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> Option<()> {
    let (vco_freq, post_div1, post_div2) = check_sys_clock_khz(system_frequency).unwrap();

    let xosc = rp2040_hal::xosc::setup_xosc_blocking(xosc_dev, EXTERNAL_CRYSTAL_FREQUENCY_HZ.Hz())
        .map_err(|_x| false)
        .unwrap();

    watchdog.enable_tick_generation((EXTERNAL_CRYSTAL_FREQUENCY_HZ / 1_000_000) as u8);

    let mut clocks = rp2040_hal::clocks::ClocksManager::new(clocks_dev);

    let pll_sys = rp2040_hal::pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        rp2040_hal::pll::PLLConfig { vco_freq, refdiv: 1, post_div1, post_div2 },
        // rp2040_hal::pll::PLLConfig { vco_freq: 1584.MHz(), refdiv: 1, post_div1: 6, post_div2: 2 },
        &mut clocks,
        resets,
    )
    .map_err(|_x| false)
    .unwrap();

    let pll_usb = rp2040_hal::pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        rp2040_hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(|_x| false)
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).map_err(|_x| false).unwrap();

    Some(())
}

pub fn check_sys_clock_khz(desired_frequency: HertzU32) -> Option<(HertzU32, u8, u8)> {
    let freq_khz = desired_frequency.to_kHz();
    let reference_divider = 1;
    let reference_freq_khz: u32 = XOSC_KHZ / reference_divider;

    for fbdiv in (16..=320).rev() {
        let vco_khz = fbdiv * reference_freq_khz;

        if !(PICO_PLL_VCO_MIN_FREQ_KHZ..=PICO_PLL_VCO_MAX_FREQ_KHZ).contains(&vco_khz) {
            continue;
        }

        for post_div1 in (1..=7).rev() {
            for postdiv_2 in (1..=post_div1).rev() {
                let out = vco_khz / (post_div1 * postdiv_2);

                if out == freq_khz && (vco_khz % (post_div1 * postdiv_2)) == 0 {
                    return Some((
                        vco_khz.kHz(),
                        post_div1.try_into().unwrap(),
                        postdiv_2.try_into().unwrap(),
                    ));
                }
            }
        }
    }

    None
}
