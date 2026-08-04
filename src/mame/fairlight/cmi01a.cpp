// license:BSD-3-Clause
// copyright-holders:Phil Bennett
/***************************************************************************

    Fairlight CMI-01A Channel Controller Card

***************************************************************************/

#include "emu.h"
#include "cmi01a.h"


#define VERBOSE     (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(CMI01A_CHANNEL_CARD, cmi01a_device, "cmi_01a", "Fairlight CMI-01A Channel Card")

cmi01a_device::cmi01a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, CMI01A_CHANNEL_CARD, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_irq_merger(*this, "cmi01a_irq")
	, m_pia(*this, "cmi01a_pia_%u", 0U)
	, m_ptm(*this, "cmi01a_ptm")
	, m_stream(nullptr)
	, m_sample_timer(nullptr)
	, m_irq_cb(*this)
	, m_current_sample(0), m_mosc(0.0), m_pitch(0), m_octave(0), m_zx_ff_clk(false), m_zx_ff(false), m_zx(false), m_gzx(false)
	, m_run(false), m_not_rstb(true), m_not_load(false), m_not_wpe(true), m_new_addr(false)
	, m_tri(false), m_permit_eload(false), m_not_eload(true), m_bcas_q1_enabled(true)
	, m_env_dir(ENV_DIR_UP), m_env(0), m_env_divider(0), m_ediv_out(false), m_eclk(false), m_env_clk(false)
	, m_wave_addr_lsb(0), m_wave_addr_msb(0), m_upper_wave_addr_load(false), m_wave_addr_msb_clock(true), m_run_load_xor(true), m_delayed_inverted_run_load(false)
	, m_ptm_c1(false), m_ptm_o1(false), m_ptm_o2(false), m_ptm_o3(false)
	, m_vol_latch(0), last_vol(0), m_flt_latch(0), m_rp(0), m_ws(0), m_dir(ENV_DIR_UP)
	, m_vr1_trim(0.0), m_vr2_trim(1.0), m_vr3_trim(0.00001), m_vr5_trim(2.0)
{
}

void cmi01a_device::device_add_mconfig(machine_config &config)
{
	PIA6821(config, m_pia[0], 0); // 6821 C6/7/8/9
	m_pia[0]->readcb1_handler().set(FUNC(cmi01a_device::tri_r));
	m_pia[0]->readpa_handler().set(FUNC(cmi01a_device::ws_dir_r));
	m_pia[0]->writepa_handler().set(FUNC(cmi01a_device::ws_dir_w));
	m_pia[0]->readpb_handler().set(FUNC(cmi01a_device::rp_r));
	m_pia[0]->writepb_handler().set(FUNC(cmi01a_device::rp_w));
	m_pia[0]->ca2_handler().set(FUNC(cmi01a_device::notload_w));
	m_pia[0]->cb2_handler().set(FUNC(cmi01a_device::run_w));
	m_pia[0]->irqa_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<0>));
	m_pia[0]->irqb_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<1>));

	PIA6821(config, m_pia[1], 0); // 6821 D6/7/8/9
	m_pia[1]->readca1_handler().set(FUNC(cmi01a_device::zx_r));
	m_pia[1]->readcb1_handler().set(FUNC(cmi01a_device::eosi_r));
	m_pia[1]->readpa_handler().set(FUNC(cmi01a_device::pitch_octave_r));
	m_pia[1]->writepa_handler().set(FUNC(cmi01a_device::pitch_octave_w));
	m_pia[1]->readpb_handler().set(FUNC(cmi01a_device::pitch_lsb_r));
	m_pia[1]->writepb_handler().set(FUNC(cmi01a_device::pitch_lsb_w));
	m_pia[1]->ca2_handler().set(FUNC(cmi01a_device::permit_eload_w));
	m_pia[1]->cb2_handler().set(FUNC(cmi01a_device::not_wpe_w));
	m_pia[1]->irqa_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<2>));
	m_pia[1]->irqb_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<3>));

	PTM6840(config, m_ptm, DERIVED_CLOCK(1, 1));
	m_ptm->o1_callback().set(FUNC(cmi01a_device::ptm_o1));
	m_ptm->o2_callback().set(FUNC(cmi01a_device::ptm_o2));
	m_ptm->o3_callback().set(FUNC(cmi01a_device::ptm_o3));
	m_ptm->irq_callback().set(m_irq_merger, FUNC(input_merger_device::in_w<4>));

	INPUT_MERGER_ANY_HIGH(config, m_irq_merger).output_handler().set(FUNC(cmi01a_device::cmi01a_irq));
}

void cmi01a_device::device_start()
{
	m_wave_ram = std::make_unique<u8[]>(0x4000);
	
	m_sample_timer = timer_alloc(FUNC(cmi01a_device::update_sample), this);
	
	m_stream = stream_alloc(0, 1, 48000);
	
	m_ptm->set_external_clocks(0, 0, 0);
	
	// Variances in components +/- 0,25%
    m_vari_vol = 0.9975 + (machine().rand() % 50 / 10000.0); 
    m_vari_flt = 0.9975 + (machine().rand() % 50 / 10000.0);
	
	save_pointer(NAME(m_wave_ram), 0x4000);
	save_item(NAME(m_current_sample));
	
	save_item(NAME(m_mosc));
	save_item(NAME(m_pitch));
	save_item(NAME(m_octave));
	
	save_item(NAME(m_zx_ff_clk));
	save_item(NAME(m_zx_ff));
	save_item(NAME(m_zx));
	save_item(NAME(m_gzx));
	
	save_item(NAME(m_run));
	save_item(NAME(m_not_rstb));
	save_item(NAME(m_not_load));
	save_item(NAME(m_not_wpe));
	save_item(NAME(m_new_addr));
	
	save_item(NAME(m_tri));
	save_item(NAME(m_permit_eload));
	save_item(NAME(m_not_eload));
	
	save_item(NAME(m_bcas_q1_enabled));
	
	save_item(NAME(m_env_dir));
	save_item(NAME(m_env));
	save_item(NAME(m_env_divider));
	save_item(NAME(m_ediv_out));
	save_item(NAME(m_envdiv_toggles));
	save_item(NAME(m_eclk));
	save_item(NAME(m_env_clk));
	
	save_item(NAME(m_wave_addr_lsb));
	save_item(NAME(m_wave_addr_msb));
	save_item(NAME(m_upper_wave_addr_load));
	save_item(NAME(m_wave_addr_msb_clock));
	save_item(NAME(m_run_load_xor));
	save_item(NAME(m_delayed_inverted_run_load));
	
	save_item(NAME(m_ptm_c1));
	save_item(NAME(m_ptm_o1));
	save_item(NAME(m_ptm_o2));
	save_item(NAME(m_ptm_o3));
	
	save_item(NAME(m_vol_latch));
	save_item(NAME(m_flt_latch));
	save_item(NAME(m_act_flt_scale));
	save_item(NAME(m_rp));
	save_item(NAME(m_ws));
	save_item(NAME(m_dir));
	
	save_item(NAME(m_ha0));
	save_item(NAME(m_ha1));
	save_item(NAME(m_hb0));
	save_item(NAME(m_hb1));
	save_item(NAME(m_hc0));
	save_item(NAME(m_hc1));
	save_item(NAME(m_ka0));
	save_item(NAME(m_ka1));
	save_item(NAME(m_ka2));
	save_item(NAME(m_kb0));
	save_item(NAME(m_kb1));
	save_item(NAME(m_kb2));
	
	save_item(NAME(m_vr1_trim));
	save_item(NAME(m_vr2_trim));
	save_item(NAME(m_vr3_trim));
	save_item(NAME(m_vr5_trim));
	
	save_item(NAME(m_dc_prev_x));
    save_item(NAME(m_dc_prev_y));
}

void cmi01a_device::device_reset()
{
	m_ptm->set_g1(1);
	m_ptm->set_g2(1);
	m_ptm->set_g3(1);

	m_current_sample = 0x80;

	m_new_addr = false;
	m_vol_latch = 0;
	last_vol = 0;
	m_flt_latch = 0;
	m_rp = 0;
	m_ws = 0;
	m_dir = 0;
	m_env = 0;
	m_not_rstb = true;

	m_ptm_o1 = 0;
	m_ptm_o2 = 0;
	m_ptm_o3 = 0;

	m_run = false;
	m_gzx = true;
	m_not_wpe = false;
	m_tri = false;
	m_permit_eload = false;

	m_eclk = false;
	m_env_clk = false;
	m_ediv_out = true;
	m_env_divider = 3;
	std::fill(std::begin(m_envdiv_toggles), std::end(m_envdiv_toggles), false);
	
	m_pitch = 0;
	m_octave = 0;
	
	m_ha0 = 0;
	m_ha1 = 0;
	m_hb0 = 0;
	m_hb1 = 0;
	m_hc0 = 0;
	m_hc1 = 0;
	m_ka0 = 1;
	m_ka1 = 0;
	m_ka2 = 0;
	m_kb0 = 1;
	m_kb1 = 0;
	m_kb2 = 0;

	m_sample_timer->adjust(attotime::never);	
	
	update_filters(m_act_cfreq);
	
	m_dc_prev_x = 0;
    m_dc_prev_y = 0;
    m_main_out = 0;
}


void cmi01a_device::sound_stream_update(sound_stream &stream)
{
	if (m_run)
	{
		for (int sampindex = 0; sampindex < stream.samples(); sampindex++)
		{
			double sample = s8(m_current_sample ^ 0x80); // -128..127

			// SSM 2045 first filter stage
			double linear_hbn = (sample + 2*m_ha0 + m_ha1 - m_ka1 * m_hb0 - m_ka2 * m_hb1) / m_ka0;
			double hbn = ssm2045_clip(linear_hbn);
			
			// SSM 2045 second filter stage 
			double linear_hcn = (hbn + 2*m_hb0 + m_hb1 - m_kb1 * m_hc0 - m_kb2 * m_hc1) / m_kb0;
			double hcn = ssm2045_clip(linear_hcn);
			
			m_ha1 = m_ha0;
			m_ha0 = sample;
			m_hb1 = m_hb0;
			m_hb0 = hbn;
			m_hc1 = m_hc0;
			m_hc0 = hcn;
			
			// SSM 2045 biased vca / output level
			double ssm2045_vca = m_env + m_vr1_trim;
			double ssm2045_out = hcn * ssm2045_vca;
			
			/********  dbx 2150 VCA & Output Stage  ********/	

			// Virtual voltage normalised to approx. 0-5 V
			double v_virt = ssm2045_out / 1000000.0; 
			
			// V-I conversion with a 10 kΩ resistor
			double i_in = v_virt / 10000.0;
			
			// VCA gain control
			double vol_gain = m_vol_latch / 255.0;
			double i_out = i_in * vol_gain;
			
			// CV-feedthrough
			double vca_thump = vol_gain * 0.0007; // 0.0001 - 0.001
			
			// I-V conversion with a 10 kΩ feedback resistor
			double v_pre_amp = i_out * 10000.0;
			
			// LF347 OpAmps (soft knee at about 1.0 (eff. 10 V))
			constexpr double drive = 0.25;
			constexpr double gain_comp = 1.0 / drive;
			double sat_signal = std::tanh((v_pre_amp + vca_thump) * drive) * gain_comp;

			// DC-offset / symmetry adjust (VR3)
			sat_signal += m_vr3_trim * (sat_signal * sat_signal);

			// VC gain adjust (VR5)
			double biased_gain = m_vr5_trim / 15.0;
			
			// Main output (biased)
			double dbx2150_out = (sat_signal * biased_gain) * m_vari_vol;
			
			// Slew rate 
			double sr_amount = 0.101; // 0.01 - 1.0   
			m_main_out = (dbx2150_out * (1.0 - sr_amount)) + (m_main_out * sr_amount);
			
			// 10 µF coupling capacitor at the output (DC-Blocker)
			double x = m_main_out;
			double y = x - m_dc_prev_x + 0.995 * m_dc_prev_y;
			m_dc_prev_x = x;
			m_dc_prev_y = y;

			stream.put(0, sampindex, y * 255);
		}
	}
	else {
		m_ha0 = m_ha1 = 0;
		m_hb0 = m_hb1 = 0;
		m_hc0 = m_hc1 = 0;
		m_dc_prev_x = 0;
		m_dc_prev_y = 0;	
	}
}

TIMER_CALLBACK_MEMBER(cmi01a_device::update_sample)
{
	m_stream->update();
	u32 mask = m_mode1 ? 0x0fff : 0x3fff; // MODE 1 (4 kB) or MODE 4 (16 kB)
	m_current_sample = m_wave_ram[((m_wave_addr_msb << 7) | m_wave_addr_lsb) & mask];
	set_wave_addr_lsb((m_wave_addr_lsb + 1) & 0x7f);
}

int cmi01a_device::notload_r()
{
	return m_not_load;
}

void cmi01a_device::notload_w(int state)
{
	set_not_load(state);
}

void cmi01a_device::pitch_octave_w(u8 data)
{
	m_pitch &= 0x0ff;
	m_pitch |= (data & 3) << 8;
	m_octave = (data >> 2) & 0x0f;
	update_filters(m_act_cfreq);
}

u8 cmi01a_device::pitch_octave_r()
{
	return ((m_pitch >> 8) & 3) | (m_octave << 2);
}

void cmi01a_device::pitch_lsb_w(u8 data)
{
	u16 new_pitch = (m_pitch & 0xf00) | data;
	if (new_pitch != m_pitch) {
		m_pitch = new_pitch;
		if (m_run) run_voice();
	}
}

u8 cmi01a_device::pitch_lsb_r()
{
	return (u8)m_pitch;
}

void cmi01a_device::rp_w(u8 data)
{
	m_rp = data;
}

u8 cmi01a_device::rp_r()
{
	return m_rp;
}

void cmi01a_device::ws_dir_w(u8 data)
{
	m_ws = data & 0x7f;
	m_dir = (data >> 7) & 1;
	try_load_upper_wave_addr();
}

u8 cmi01a_device::ws_dir_r()
{
	return m_ws | (m_dir << 7);
}

int cmi01a_device::tri_r()
{
	return m_tri;
}

void cmi01a_device::cmi01a_irq(int state)
{
	m_irq_cb(state ? ASSERT_LINE : CLEAR_LINE);
}

void cmi01a_device::permit_eload_w(int state)
{
	m_permit_eload = state;
	update_not_eload();
}

void cmi01a_device::run_voice()
{
	double cfreq = ((0x800 | (m_pitch << 1)) * m_mosc) / 4096.0;

	//Octave register enabled?
	if (!BIT(m_octave, 3))
		cfreq /= (double)(2 << ((7 ^ m_octave) & 7));

	cfreq /= 16.0;
	
	/*double cfreq = (
		!BIT(m_octave, 3) ?
			((((0x800 | (m_pitch << 1)) * m_mosc) / 4096.0) / (double)(2 << ((7 ^ m_octave) & 7))) / 16.0 :
			((((0x800 | (m_pitch << 1)) * m_mosc) / 4096.0) / 16.0)
	);*/

	// Update filter coefficients for key tracking
	m_act_cfreq = cfreq;
	update_filters(m_act_cfreq);
	
	attotime updated_cfreq = attotime::from_hz(cfreq);
	m_sample_timer->adjust(updated_cfreq, 0, updated_cfreq);
}

void cmi01a_device::run_w(int state)
{
	bool old_run = m_run;
	m_run = state;

	if (old_run != m_run)
		update_rstb_pulser();

	m_stream->update();

	/* RUN */
	if (!old_run && m_run)
	{
		run_voice();

		m_ptm->set_g1(0); // Loop
		m_ptm->set_g2(0); // Damping
		m_ptm->set_g3(0); // Attack
	}

	if (old_run && !m_run)
	{
		m_sample_timer->adjust(attotime::never);
		m_current_sample = 0x80;

		m_ptm->set_g1(1);
		m_ptm->set_g2(1);
		m_ptm->set_g3(1);

		set_zx_flipflop_state(false);
	}
}

inline void cmi01a_device::update_rstb_pulser()
{
	set_run_load_xor(m_run != !m_not_load);
}

void cmi01a_device::set_run_load_xor(const bool run_load_xor)
{
	if (run_load_xor == m_run_load_xor)
		return;

	m_run_load_xor = run_load_xor;
	m_new_addr = true;

	// pulse /RSTB low
	m_not_rstb = false;
	set_gzx(true);
	set_wave_addr_lsb(0);
	set_wave_addr_msb(0x80 | m_ws);

	// return /RSTB high
	m_not_rstb = true;
	set_gzx(false);
}

void cmi01a_device::update_bcas_q1_enable()
{
	const bool old_enable = m_bcas_q1_enabled;
	m_bcas_q1_enabled = (m_zx_ff == m_ptm_o1);

	if (!old_enable && m_bcas_q1_enabled)
	{
		if (m_not_load)
			m_ptm->set_ext_clock(0, clock() / 8.0);
		else
			m_ptm->set_ext_clock(0, 0.0);
		m_ptm->set_ext_clock(1, clock() / 4.0);
		m_ptm->set_ext_clock(2, clock() / 4.0);
	}
	else if (old_enable && !m_bcas_q1_enabled)
	{
		m_ptm->set_ext_clock(0, 0.0);
		m_ptm->set_ext_clock(1, 0.0);
		m_ptm->set_ext_clock(2, 0.0);
	}
}

void cmi01a_device::set_zx_flipflop_clock(const bool zx_ff_clk)
{
	if (zx_ff_clk == m_zx_ff_clk)
		return;

	m_zx_ff_clk = zx_ff_clk;

	if (m_zx_ff_clk && m_run)
		set_zx_flipflop_state(m_ptm_o1);
}

void cmi01a_device::set_zx_flipflop_state(const bool zx_ff)
{
	if (zx_ff == m_zx_ff)
		return;

	m_zx_ff = zx_ff;

	update_bcas_q1_enable();
	pulse_zcint();
}

inline void cmi01a_device::pulse_zcint()
{
	// pulse /ZCINT low
	m_pia[0]->ca1_w(1);
	set_gzx(true);

	// return /ZCINT high
	m_pia[0]->ca1_w(0);
	set_gzx(false);
}

void cmi01a_device::set_not_load(const bool not_load)
{
	if (not_load == m_not_load)
		return;

	m_not_load = not_load;
	update_rstb_pulser();
	update_ptm_c1();
}

void cmi01a_device::set_gzx(const bool gzx)
{
	if (gzx == m_gzx)
		return;

	m_gzx = gzx;
	update_upper_wave_addr_load();
	update_not_eload();
	if (m_gzx)
		set_envelope_dir(m_dir);
}

inline void cmi01a_device::update_not_eload()
{
	set_not_eload(!(m_permit_eload && m_gzx));
}

void cmi01a_device::set_not_eload(const bool not_eload)
{
	if (not_eload == m_not_eload)
		return;

	m_not_eload = not_eload;
	try_load_envelope();
}

inline void cmi01a_device::try_load_envelope()
{
	if (m_not_eload)
		return;

	set_envelope(m_rp);
}

void cmi01a_device::set_envelope(const u8 env)
{
	if (env == m_env)
		return;

	m_env = env;
	update_envelope_divider();
	update_envelope_tri();
}

void cmi01a_device::update_envelope_divider()
{
	if (m_env_dir == ENV_DIR_UP)
		m_env_divider = (~m_env >> 2) & 0x3c;
	else
		m_env_divider = (m_env >> 2) & 0x3c;
		
	m_env_divider |= 0x03;
}

void cmi01a_device::set_envelope_dir(const int env_dir)
{
	if (env_dir == m_env_dir)
		return;

	m_env_dir = env_dir;
	update_envelope_divider();
	update_envelope_tri();
}

void cmi01a_device::update_envelope_clock()
{
	const bool old_eclk = m_eclk;
	m_eclk = (m_ptm_o2 && m_zx_ff) || (m_ptm_o3 && !m_zx_ff);

	if (old_eclk == m_eclk)
		return;

	tick_ediv();

	const bool old_env_clk = m_env_clk;
	m_env_clk = ((m_not_load && m_eclk) || (!m_not_load && m_ediv_out));

	if (!old_env_clk && m_env_clk)
		clock_envelope();
}

void cmi01a_device::clock_envelope()
{
	if (m_tri)
		return;

	m_stream->update();
	
	if (m_env_dir == ENV_DIR_DOWN)
		m_env--;
	else
		m_env++;
		
	update_envelope_divider();
	update_envelope_tri();
}

void cmi01a_device::tick_ediv()
{
	const bool envdiv_enable_a = m_eclk;
	const bool envdiv_enable_b = m_eclk && m_envdiv_toggles[0];
	const bool envdiv_enable_c = m_eclk && m_envdiv_toggles[0] && m_envdiv_toggles[1];
	const bool envdiv_enable_d = m_eclk && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2];
	const bool envdiv_enable_e = m_eclk && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2] && m_envdiv_toggles[3];
	const bool envdiv_enable_f = m_eclk && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2] && m_envdiv_toggles[3] && m_envdiv_toggles[4];

	if (envdiv_enable_f)
		m_envdiv_toggles[5] = !m_envdiv_toggles[5];
	if (envdiv_enable_e)
		m_envdiv_toggles[4] = !m_envdiv_toggles[4];
	if (envdiv_enable_d)
		m_envdiv_toggles[3] = !m_envdiv_toggles[3];
	if (envdiv_enable_c)
		m_envdiv_toggles[2] = !m_envdiv_toggles[2];
	if (envdiv_enable_b)
		m_envdiv_toggles[1] = !m_envdiv_toggles[1];
	if (envdiv_enable_a)
		m_envdiv_toggles[0] = !m_envdiv_toggles[0];

	const bool envdiv_out_f = m_eclk && BIT(m_env_divider, 5) && !m_envdiv_toggles[0];
	const bool envdiv_out_e = m_eclk && BIT(m_env_divider, 4) && m_envdiv_toggles[0] && !m_envdiv_toggles[1];
	const bool envdiv_out_d = m_eclk && BIT(m_env_divider, 3) && m_envdiv_toggles[0] && m_envdiv_toggles[1] && !m_envdiv_toggles[2];
	const bool envdiv_out_c = m_eclk && BIT(m_env_divider, 2) && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2] && !m_envdiv_toggles[3];
	const bool envdiv_out_b = m_eclk && BIT(m_env_divider, 1) && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2] && m_envdiv_toggles[3] && !m_envdiv_toggles[4];
	const bool envdiv_out_a = m_eclk && BIT(m_env_divider, 0) && m_envdiv_toggles[0] && m_envdiv_toggles[1] && m_envdiv_toggles[2] && m_envdiv_toggles[3] && m_envdiv_toggles[4] && !m_envdiv_toggles[5];

	m_ediv_out = !(envdiv_out_f || envdiv_out_e || envdiv_out_d || envdiv_out_c || envdiv_out_b || envdiv_out_a);
}

void cmi01a_device::update_envelope_tri()
{
	if (m_env_dir == ENV_DIR_DOWN)
		m_tri = (m_env == 0x00);
	else
		m_tri = (m_env == 0xff);

	m_pia[0]->cb1_w(m_tri);
}

void cmi01a_device::not_wpe_w(int state)
{
	if (state == m_not_wpe)
		return;

	m_not_wpe = state;
	update_upper_wave_addr_load();
}

inline double cmi01a_device::ssm2045_clip(double x) const
{
	// Preserving low level signals 
	constexpr double pre = 1.156;
	constexpr double post = 1.0 / pre;
	
	// Proportional saturation
	return std::tanh(x * pre / m_act_flt_scale) * m_act_flt_scale * post;	
}

void cmi01a_device::update_filters(double dac_rate) 
{
    constexpr double two_pi = 2 * M_PI;

	// Calibrated using the graph on page 133 of the CMI Mainframe Service Manual
	//constexpr double base_freq = 3821.0; // Adjusted to match the 16 Hz at the zero setting
	constexpr double base_freq = 5120.0; // Adjusted to more closely match the 16 Hz at the zero setting
	constexpr double sec_cutoff = 14000.0;			
	constexpr double max_cutoff = 23581.0; // -6 bB at 21,6 kHz 
	
    // Filter ADC input level
    int fval = (m_octave << 5) + m_flt_latch;
		
	// Filter clipping
	double cutoff_scale = 550.0 - (double(fval) / 511.0) * 150.0;
	double env_amount = 1.0 + (m_env / 255.0) * 0.8; 
    double flt_scale = cutoff_scale / env_amount;
	m_act_flt_scale = std::clamp(flt_scale, 334.0, 600.0);
		
	// Filter cutoff frequency (biased)
	double fc = (base_freq * pow(1.02162, (fval - 256)) * m_vr2_trim) * m_vari_flt; 
	
	// Limit to max. cutoff frequency
	double max_fc = std::min(max_cutoff, (dac_rate / 2.0) * 0.99);
	fc = std::min(fc, max_fc);		
	
    // -6dB cutoff frequency
    double f0 = fc * 0.916;
	
	// Secondary filter around there for when the first filter is high
    double fc_sec = (fc > sec_cutoff) ? sec_cutoff : fc;
	
    // Precompute angular frequencies
    double w1 = two_pi * fc_sec;
    double w2 = w1 * 1.22474487139159;
	
    /* Predefined constants scaled by sqrt(c1*c2 / (c3*c4)), 
	   the ratio between the two cutoff frequencies in the cmi01 configuration of the SSM2045 */
    constexpr double a1 = 1.81659021245849;
    constexpr double a2 = 1.48323969741913;
	
	/* Two stages of order-2 lowpass filters with fixed Q */	
	
    // Compute lowpass filter parameters H(s) = 1/(m0 * s**2 + m1 * s + 1) 
    double ma0 = a1 / w1;
    double ma1 = 1 / (w1 * w1);
    double mb0 = a2 / w2;
    double mb1 = 1 / (w2 * w2);

    // Convert to z, wrap around f0
    double zc = two_pi * f0 / tan(M_PI * f0 / dac_rate);
	
    // Compute z-domain coefficients
    double zc_sq = zc * zc;
    double za0 = ma1 * zc_sq;
    double za1 = ma0 * zc;
    double zb0 = mb1 * zc_sq;
    double zb1 = mb0 * zc;
	
	// Filter coefficients H(z) = (1 + 2 * z-1 + z-2) / (k0 + k1 * z-1 + k2 * z-2)
	
    // First filter coefficients
    m_ka0 = za0 + za1 + 1;
    m_ka1 = -2 * za0 + 2;
    m_ka2 = za0 - za1 + 1;
	
    // Second filter coefficients
    m_kb0 = zb0 + zb1 + 1;
    m_kb1 = -2 * zb0 + 2;
    m_kb2 = zb0 - zb1 + 1;
}

inline void cmi01a_device::update_upper_wave_addr_load()
{
	const bool c10_and_out = (!m_not_wpe && m_gzx);
	set_upper_wave_addr_load(c10_and_out || !m_not_rstb);
}

inline void cmi01a_device::set_upper_wave_addr_load(const bool upper_wave_addr_load)
{
	if (upper_wave_addr_load == m_upper_wave_addr_load)
		return;

	m_upper_wave_addr_load = upper_wave_addr_load;
	try_load_upper_wave_addr();
}

inline void cmi01a_device::try_load_upper_wave_addr()
{
	if (!m_upper_wave_addr_load)
		return;

	set_wave_addr_msb(0x80 | m_ws);
}

void cmi01a_device::set_wave_addr_lsb(const u8 wave_addr_lsb)
{
	if (wave_addr_lsb == m_wave_addr_lsb)
		return;

	m_wave_addr_lsb = wave_addr_lsb;
	set_zx(BIT(m_wave_addr_lsb, 6));
}

void cmi01a_device::set_wave_addr_msb(const u8 wave_addr_msb)
{
	if (wave_addr_msb == m_wave_addr_msb)
		return;
	u8 act_msb = wave_addr_msb;
	
	// In MODE 1 EOSI triggers at 4 kB
    if (m_mode1 && (wave_addr_msb & 0x20)) act_msb |= 0x80;

    m_wave_addr_msb = act_msb;
    m_pia[1]->cb1_w(BIT(m_wave_addr_msb, 7));
}

void cmi01a_device::set_wave_addr_msb_clock(const bool wave_addr_msb_clock)
{
	if (wave_addr_msb_clock == m_wave_addr_msb_clock)
		return;

	m_wave_addr_msb_clock = wave_addr_msb_clock;
	if (m_wave_addr_msb_clock)
		set_wave_addr_msb(m_wave_addr_msb + 1);
}

void cmi01a_device::set_zx(const bool zx)
{
	if (zx == m_zx)
		return;

	m_zx = zx;
	set_wave_addr_msb_clock(!(!m_not_load && m_zx));
	m_pia[1]->ca1_w(m_zx);
	set_zx_flipflop_clock(!m_zx);
	update_ptm_c1();
}

void cmi01a_device::update_ptm_c1()
{
	const bool old_ptm_c1 = m_ptm_c1;
	m_ptm_c1 = !m_not_load && !m_zx;
	if (old_ptm_c1 != m_ptm_c1)
		m_ptm->set_c1(m_ptm_c1);
}

void cmi01a_device::ptm_o1(int state)
{
	m_ptm_o1 = state;
	update_bcas_q1_enable();
}

void cmi01a_device::ptm_o2(int state)
{
	m_ptm_o2 = state;
	update_envelope_clock();
}

void cmi01a_device::ptm_o3(int state)
{
	m_ptm_o3 = state;
	update_envelope_clock();
}

int cmi01a_device::eosi_r()
{
	return BIT(m_wave_addr_msb, 7);
}

int cmi01a_device::zx_r()
{
	return BIT(m_wave_addr_lsb, 6);
}

void cmi01a_device::write(offs_t offset, u8 data)
{
	switch (offset)
	{
		case 0x0:
		{
			if (m_new_addr)
				m_new_addr = false;
			u32 mask = m_mode1 ? 0x0fff : 0x3fff;
			m_wave_ram[((m_wave_addr_msb << 7) | m_wave_addr_lsb) & mask] = data;
			set_wave_addr_lsb((m_wave_addr_lsb + 1) & 0x7f);
			break;
		}

		/* case 0x1:				
			FF = Note on 00 = Note off (CMI01 card)
			break;
		 */
		 
		case 0x3:
			set_envelope_dir(ENV_DIR_DOWN);
			break;

		case 0x4:
			set_envelope_dir(ENV_DIR_UP);
			break;

		case 0x5:
			m_vol_latch = data;
			//logerror("VOL LATCH: %u", data);
			break;

		case 0x6:
			m_flt_latch = data;
			update_filters(m_act_cfreq);
			break;

		case 0x8: case 0x9: case 0xa: case 0xb:
			m_pia[0]->write(offset & 3, data);
			break;

		case 0xc: case 0xd: case 0xe: case 0xf:
			m_pia[1]->write((BIT(offset, 0) << 1) | BIT(offset, 1), data);
			break;

		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		{
			/* PTM addressing is a little funky */
			int a0 = offset & 1;
			int a1;
			int a2 = BIT(offset, 1);
			if (m_mode1) a1 = BIT(offset, 2); 
			else a1 = (m_ptm_o1 && BIT(offset, 3)) || (!BIT(offset, 3) && BIT(offset, 2));

			m_ptm->write((a2 << 2) | (a1 << 1) | a0, data);
			break;
		}
			
		case 0x1a: // MODE 1 hack
		{
			if (!m_mode1) {
				m_mode1 = true;
				m_zx_ff = false;
				m_ptm_o1 = false;
				m_bcas_q1_enabled = false;
				update_bcas_q1_enable(); 
				m_env_dir = ENV_DIR_UP;
			}
			break;
		}
		
		//case 0x1b:  MODE 1 sets FF most of the time
		case 0x1b: // MODE 4 
			if (m_mode1) {
				m_mode1 = false;	
			}
			break;
			
		default:
			//logerror("%s: Register write at offset %02X (Mode1=%d)\n", machine().describe_context(), offset, m_mode1);
			break;
	}
}

u8 cmi01a_device::read(offs_t offset)
{
	if (machine().side_effects_disabled())
		return 0;

	u8 data = 0;

	switch (offset)
	{
		case 0x0:
		{
			u32 mask = m_mode1 ? 0x0fff : 0x3fff;
			data = m_wave_ram[((m_wave_addr_msb << 7) | m_wave_addr_lsb) & mask];
			if (!m_new_addr)
			{
				set_wave_addr_lsb((m_wave_addr_lsb + 1) & 0x7f);
			}
			m_new_addr = false;
			break;
		}
/*		
		case 0x02: // EOSI status register?
			break
*/		
		case 0x3:
			set_envelope_dir(ENV_DIR_DOWN);	
			break;

		case 0x4:
			set_envelope_dir(ENV_DIR_UP);
			break;

		case 0x5:
			data = 0xff;
			break;

		case 0x8: case 0x9: case 0xa: case 0xb:
			data = m_pia[0]->read(offset & 3);
			break;

		case 0xc: case 0xd: case 0xe: case 0xf:
			data = m_pia[1]->read((BIT(offset, 0) << 1) | BIT(offset, 1));
			break;


		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		{
			int a0 = offset & 1;
			int a1;
			int a2 = BIT(offset, 1);

			if (m_mode1) a1 = BIT(offset, 2); 
			else a1 = (m_ptm_o1 && BIT(offset, 3)) || (!BIT(offset, 3) && BIT(offset, 2));
			
			data = m_ptm->read((a2 << 2) | (a1 << 1) | a0);
			break;
		}

		default:
			break;
	}
	return data;
}
