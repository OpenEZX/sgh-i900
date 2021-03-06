/*
 * Copyright 2007-8 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie
 *          Alex Deucher
 */
#include "drmP.h"
#include "drm_crtc_helper.h"
#include "radeon_drm.h"
#include "radeon.h"
#include "atom.h"


static void radeon_legacy_rmx_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	int    xres = mode->hdisplay;
	int    yres = mode->vdisplay;
	bool   hscale = true, vscale = true;
	int    hsync_wid;
	int    vsync_wid;
	int    hsync_start;
	uint32_t scale, inc;
	uint32_t fp_horz_stretch, fp_vert_stretch, crtc_more_cntl, fp_horz_vert_active;
	uint32_t fp_h_sync_strt_wid, fp_v_sync_strt_wid, fp_crtc_h_total_disp, fp_crtc_v_total_disp;
	struct radeon_native_mode *native_mode = &radeon_encoder->native_mode;

	DRM_DEBUG("\n");

	fp_vert_stretch = RREG32(RADEON_FP_VERT_STRETCH) &
		(RADEON_VERT_STRETCH_RESERVED |
		 RADEON_VERT_AUTO_RATIO_INC);
	fp_horz_stretch = RREG32(RADEON_FP_HORZ_STRETCH) &
		(RADEON_HORZ_FP_LOOP_STRETCH |
		 RADEON_HORZ_AUTO_RATIO_INC);

	crtc_more_cntl = 0;
	if ((rdev->family == CHIP_RS100) ||
	    (rdev->family == CHIP_RS200)) {
		/* This is to workaround the asic bug for RMX, some versions
		   of BIOS dosen't have this register initialized correctly. */
		crtc_more_cntl |= RADEON_CRTC_H_CUTOFF_ACTIVE_EN;
	}


	fp_crtc_h_total_disp = ((((mode->crtc_htotal / 8) - 1) & 0x3ff)
				| ((((mode->crtc_hdisplay / 8) - 1) & 0x1ff) << 16));

	hsync_wid = (mode->crtc_hsync_end - mode->crtc_hsync_start) / 8;
	if (!hsync_wid)
		hsync_wid = 1;
	hsync_start = mode->crtc_hsync_start - 8;

	fp_h_sync_strt_wid = ((hsync_start & 0x1fff)
			      | ((hsync_wid & 0x3f) << 16)
			      | ((mode->flags & DRM_MODE_FLAG_NHSYNC)
				 ? RADEON_CRTC_H_SYNC_POL
				 : 0));

	fp_crtc_v_total_disp = (((mode->crtc_vtotal - 1) & 0xffff)
				| ((mode->crtc_vdisplay - 1) << 16));

	vsync_wid = mode->crtc_vsync_end - mode->crtc_vsync_start;
	if (!vsync_wid)
		vsync_wid = 1;

	fp_v_sync_strt_wid = (((mode->crtc_vsync_start - 1) & 0xfff)
			      | ((vsync_wid & 0x1f) << 16)
			      | ((mode->flags & DRM_MODE_FLAG_NVSYNC)
				 ? RADEON_CRTC_V_SYNC_POL
				 : 0));

	fp_horz_vert_active = 0;

	if (native_mode->panel_xres == 0 ||
	    native_mode->panel_yres == 0) {
		hscale = false;
		vscale = false;
	} else {
		if (xres > native_mode->panel_xres)
			xres = native_mode->panel_xres;
		if (yres > native_mode->panel_yres)
			yres = native_mode->panel_yres;

		if (xres == native_mode->panel_xres)
			hscale = false;
		if (yres == native_mode->panel_yres)
			vscale = false;
	}

	if (radeon_encoder->flags & RADEON_USE_RMX) {
		if (radeon_encoder->rmx_type != RMX_CENTER) {
			if (!hscale)
				fp_horz_stretch |= ((xres/8-1) << 16);
			else {
				inc = (fp_horz_stretch & RADEON_HORZ_AUTO_RATIO_INC) ? 1 : 0;
				scale = ((xres + inc) * RADEON_HORZ_STRETCH_RATIO_MAX)
					/ native_mode->panel_xres + 1;
				fp_horz_stretch |= (((scale) & RADEON_HORZ_STRETCH_RATIO_MASK) |
						    RADEON_HORZ_STRETCH_BLEND |
						    RADEON_HORZ_STRETCH_ENABLE |
						    ((native_mode->panel_xres/8-1) << 16));
			}

			if (!vscale)
				fp_vert_stretch |= ((yres-1) << 12);
			else {
				inc = (fp_vert_stretch & RADEON_VERT_AUTO_RATIO_INC) ? 1 : 0;
				scale = ((yres + inc) * RADEON_VERT_STRETCH_RATIO_MAX)
					/ native_mode->panel_yres + 1;
				fp_vert_stretch |= (((scale) & RADEON_VERT_STRETCH_RATIO_MASK) |
						    RADEON_VERT_STRETCH_ENABLE |
						    RADEON_VERT_STRETCH_BLEND |
						    ((native_mode->panel_yres-1) << 12));
			}
		} else if (radeon_encoder->rmx_type == RMX_CENTER) {
			int    blank_width;

			fp_horz_stretch |= ((xres/8-1) << 16);
			fp_vert_stretch |= ((yres-1) << 12);

			crtc_more_cntl |= (RADEON_CRTC_AUTO_HORZ_CENTER_EN |
					   RADEON_CRTC_AUTO_VERT_CENTER_EN);

			blank_width = (mode->crtc_hblank_end - mode->crtc_hblank_start) / 8;
			if (blank_width > 110)
				blank_width = 110;

			fp_crtc_h_total_disp = (((blank_width) & 0x3ff)
						| ((((mode->crtc_hdisplay / 8) - 1) & 0x1ff) << 16));

			hsync_wid = (mode->crtc_hsync_end - mode->crtc_hsync_start) / 8;
			if (!hsync_wid)
				hsync_wid = 1;

			fp_h_sync_strt_wid = ((((mode->crtc_hsync_start - mode->crtc_hblank_start) / 8) & 0x1fff)
					      | ((hsync_wid & 0x3f) << 16)
					      | ((mode->flags & DRM_MODE_FLAG_NHSYNC)
						 ? RADEON_CRTC_H_SYNC_POL
						 : 0));

			fp_crtc_v_total_disp = (((mode->crtc_vblank_end - mode->crtc_vblank_start) & 0xffff)
						| ((mode->crtc_vdisplay - 1) << 16));

			vsync_wid = mode->crtc_vsync_end - mode->crtc_vsync_start;
			if (!vsync_wid)
				vsync_wid = 1;

			fp_v_sync_strt_wid = ((((mode->crtc_vsync_start - mode->crtc_vblank_start) & 0xfff)
					       | ((vsync_wid & 0x1f) << 16)
					       | ((mode->flags & DRM_MODE_FLAG_NVSYNC)
						  ? RADEON_CRTC_V_SYNC_POL
						  : 0)));

			fp_horz_vert_active = (((native_mode->panel_yres) & 0xfff) |
					       (((native_mode->panel_xres / 8) & 0x1ff) << 16));
		}
	} else {
		fp_horz_stretch |= ((xres/8-1) << 16);
		fp_vert_stretch |= ((yres-1) << 12);
	}

	WREG32(RADEON_FP_HORZ_STRETCH,      fp_horz_stretch);
	WREG32(RADEON_FP_VERT_STRETCH,      fp_vert_stretch);
	WREG32(RADEON_CRTC_MORE_CNTL,       crtc_more_cntl);
	WREG32(RADEON_FP_HORZ_VERT_ACTIVE,  fp_horz_vert_active);
	WREG32(RADEON_FP_H_SYNC_STRT_WID,   fp_h_sync_strt_wid);
	WREG32(RADEON_FP_V_SYNC_STRT_WID,   fp_v_sync_strt_wid);
	WREG32(RADEON_FP_CRTC_H_TOTAL_DISP, fp_crtc_h_total_disp);
	WREG32(RADEON_FP_CRTC_V_TOTAL_DISP, fp_crtc_v_total_disp);

}

static void radeon_legacy_lvds_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t lvds_gen_cntl, lvds_pll_cntl, pixclks_cntl, disp_pwr_man;
	int panel_pwr_delay = 2000;
	DRM_DEBUG("\n");

	if (radeon_encoder->enc_priv) {
		if (rdev->is_atom_bios) {
			struct radeon_encoder_atom_dig *lvds = radeon_encoder->enc_priv;
			panel_pwr_delay = lvds->panel_pwr_delay;
		} else {
			struct radeon_encoder_lvds *lvds = radeon_encoder->enc_priv;
			panel_pwr_delay = lvds->panel_pwr_delay;
		}
	}

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		disp_pwr_man = RREG32(RADEON_DISP_PWR_MAN);
		disp_pwr_man |= RADEON_AUTO_PWRUP_EN;
		WREG32(RADEON_DISP_PWR_MAN, disp_pwr_man);
		lvds_pll_cntl = RREG32(RADEON_LVDS_PLL_CNTL);
		lvds_pll_cntl |= RADEON_LVDS_PLL_EN;
		WREG32(RADEON_LVDS_PLL_CNTL, lvds_pll_cntl);
		udelay(1000);

		lvds_pll_cntl = RREG32(RADEON_LVDS_PLL_CNTL);
		lvds_pll_cntl &= ~RADEON_LVDS_PLL_RESET;
		WREG32(RADEON_LVDS_PLL_CNTL, lvds_pll_cntl);

		lvds_gen_cntl = RREG32(RADEON_LVDS_GEN_CNTL);
		lvds_gen_cntl |= (RADEON_LVDS_ON | RADEON_LVDS_EN | RADEON_LVDS_DIGON | RADEON_LVDS_BLON);
		lvds_gen_cntl &= ~(RADEON_LVDS_DISPLAY_DIS);
		udelay(panel_pwr_delay * 1000);
		WREG32(RADEON_LVDS_GEN_CNTL, lvds_gen_cntl);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		pixclks_cntl = RREG32_PLL(RADEON_PIXCLKS_CNTL);
		WREG32_PLL_P(RADEON_PIXCLKS_CNTL, 0, ~RADEON_PIXCLK_LVDS_ALWAYS_ONb);
		lvds_gen_cntl = RREG32(RADEON_LVDS_GEN_CNTL);
		lvds_gen_cntl |= RADEON_LVDS_DISPLAY_DIS;
		lvds_gen_cntl &= ~(RADEON_LVDS_ON | RADEON_LVDS_BLON | RADEON_LVDS_EN | RADEON_LVDS_DIGON);
		udelay(panel_pwr_delay * 1000);
		WREG32(RADEON_LVDS_GEN_CNTL, lvds_gen_cntl);
		WREG32_PLL(RADEON_PIXCLKS_CNTL, pixclks_cntl);
		break;
	}

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
	else
		radeon_combios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
}

static void radeon_legacy_lvds_prepare(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
	radeon_legacy_lvds_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void radeon_legacy_lvds_commit(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	radeon_legacy_lvds_dpms(encoder, DRM_MODE_DPMS_ON);
	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, false);
	else
		radeon_combios_output_lock(encoder, false);
}

static void radeon_legacy_lvds_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_crtc *radeon_crtc = to_radeon_crtc(encoder->crtc);
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t lvds_pll_cntl, lvds_gen_cntl, lvds_ss_gen_cntl;

	DRM_DEBUG("\n");

	if (radeon_crtc->crtc_id == 0)
		radeon_legacy_rmx_mode_set(encoder, mode, adjusted_mode);

	lvds_pll_cntl = RREG32(RADEON_LVDS_PLL_CNTL);
	lvds_pll_cntl &= ~RADEON_LVDS_PLL_EN;

	lvds_ss_gen_cntl = RREG32(RADEON_LVDS_SS_GEN_CNTL);
	if ((!rdev->is_atom_bios)) {
		struct radeon_encoder_lvds *lvds = (struct radeon_encoder_lvds *)radeon_encoder->enc_priv;
		if (lvds) {
			DRM_DEBUG("bios LVDS_GEN_CNTL: 0x%x\n", lvds->lvds_gen_cntl);
			lvds_gen_cntl = lvds->lvds_gen_cntl;
			lvds_ss_gen_cntl &= ~((0xf << RADEON_LVDS_PWRSEQ_DELAY1_SHIFT) |
					      (0xf << RADEON_LVDS_PWRSEQ_DELAY2_SHIFT));
			lvds_ss_gen_cntl |= ((lvds->panel_digon_delay << RADEON_LVDS_PWRSEQ_DELAY1_SHIFT) |
					     (lvds->panel_blon_delay << RADEON_LVDS_PWRSEQ_DELAY2_SHIFT));
		} else
			lvds_gen_cntl = RREG32(RADEON_LVDS_GEN_CNTL);
	} else
		lvds_gen_cntl = RREG32(RADEON_LVDS_GEN_CNTL);
	lvds_gen_cntl |= RADEON_LVDS_DISPLAY_DIS;
	lvds_gen_cntl &= ~(RADEON_LVDS_ON |
			   RADEON_LVDS_BLON |
			   RADEON_LVDS_EN |
			   RADEON_LVDS_RST_FM);

	if (ASIC_IS_R300(rdev))
		lvds_pll_cntl &= ~(R300_LVDS_SRC_SEL_MASK);

	if (radeon_crtc->crtc_id == 0) {
		if (ASIC_IS_R300(rdev)) {
			if (radeon_encoder->flags & RADEON_USE_RMX)
				lvds_pll_cntl |= R300_LVDS_SRC_SEL_RMX;
		} else
			lvds_gen_cntl &= ~RADEON_LVDS_SEL_CRTC2;
	} else {
		if (ASIC_IS_R300(rdev))
			lvds_pll_cntl |= R300_LVDS_SRC_SEL_CRTC2;
		else
			lvds_gen_cntl |= RADEON_LVDS_SEL_CRTC2;
	}

	WREG32(RADEON_LVDS_GEN_CNTL, lvds_gen_cntl);
	WREG32(RADEON_LVDS_PLL_CNTL, lvds_pll_cntl);
	WREG32(RADEON_LVDS_SS_GEN_CNTL, lvds_ss_gen_cntl);

	if (rdev->family == CHIP_RV410)
		WREG32(RADEON_CLOCK_CNTL_INDEX, 0);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
	else
		radeon_combios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
}

static bool radeon_legacy_lvds_mode_fixup(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted_mode)
{
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	radeon_encoder->flags &= ~RADEON_USE_RMX;

	if (radeon_encoder->rmx_type != RMX_OFF)
		radeon_rmx_mode_fixup(encoder, mode, adjusted_mode);

	return true;
}

static const struct drm_encoder_helper_funcs radeon_legacy_lvds_helper_funcs = {
	.dpms = radeon_legacy_lvds_dpms,
	.mode_fixup = radeon_legacy_lvds_mode_fixup,
	.prepare = radeon_legacy_lvds_prepare,
	.mode_set = radeon_legacy_lvds_mode_set,
	.commit = radeon_legacy_lvds_commit,
};


static const struct drm_encoder_funcs radeon_legacy_lvds_enc_funcs = {
	.destroy = radeon_enc_destroy,
};

static bool radeon_legacy_primary_dac_mode_fixup(struct drm_encoder *encoder,
						 struct drm_display_mode *mode,
						 struct drm_display_mode *adjusted_mode)
{

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	return true;
}

static void radeon_legacy_primary_dac_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t crtc_ext_cntl = RREG32(RADEON_CRTC_EXT_CNTL);
	uint32_t dac_cntl = RREG32(RADEON_DAC_CNTL);
	uint32_t dac_macro_cntl = RREG32(RADEON_DAC_MACRO_CNTL);

	DRM_DEBUG("\n");

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		crtc_ext_cntl |= RADEON_CRTC_CRT_ON;
		dac_cntl &= ~RADEON_DAC_PDWN;
		dac_macro_cntl &= ~(RADEON_DAC_PDWN_R |
				    RADEON_DAC_PDWN_G |
				    RADEON_DAC_PDWN_B);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		crtc_ext_cntl &= ~RADEON_CRTC_CRT_ON;
		dac_cntl |= RADEON_DAC_PDWN;
		dac_macro_cntl |= (RADEON_DAC_PDWN_R |
				   RADEON_DAC_PDWN_G |
				   RADEON_DAC_PDWN_B);
		break;
	}

	WREG32(RADEON_CRTC_EXT_CNTL, crtc_ext_cntl);
	WREG32(RADEON_DAC_CNTL, dac_cntl);
	WREG32(RADEON_DAC_MACRO_CNTL, dac_macro_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
	else
		radeon_combios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
}

static void radeon_legacy_primary_dac_prepare(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
	radeon_legacy_primary_dac_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void radeon_legacy_primary_dac_commit(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	radeon_legacy_primary_dac_dpms(encoder, DRM_MODE_DPMS_ON);

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, false);
	else
		radeon_combios_output_lock(encoder, false);
}

static void radeon_legacy_primary_dac_mode_set(struct drm_encoder *encoder,
					       struct drm_display_mode *mode,
					       struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_crtc *radeon_crtc = to_radeon_crtc(encoder->crtc);
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t disp_output_cntl, dac_cntl, dac2_cntl, dac_macro_cntl;

	DRM_DEBUG("\n");

	if (radeon_crtc->crtc_id == 0)
		radeon_legacy_rmx_mode_set(encoder, mode, adjusted_mode);

	if (radeon_crtc->crtc_id == 0) {
		if (rdev->family == CHIP_R200 || ASIC_IS_R300(rdev)) {
			disp_output_cntl = RREG32(RADEON_DISP_OUTPUT_CNTL) &
				~(RADEON_DISP_DAC_SOURCE_MASK);
			WREG32(RADEON_DISP_OUTPUT_CNTL, disp_output_cntl);
		} else {
			dac2_cntl = RREG32(RADEON_DAC_CNTL2)  & ~(RADEON_DAC2_DAC_CLK_SEL);
			WREG32(RADEON_DAC_CNTL2, dac2_cntl);
		}
	} else {
		if (rdev->family == CHIP_R200 || ASIC_IS_R300(rdev)) {
			disp_output_cntl = RREG32(RADEON_DISP_OUTPUT_CNTL) &
				~(RADEON_DISP_DAC_SOURCE_MASK);
			disp_output_cntl |= RADEON_DISP_DAC_SOURCE_CRTC2;
			WREG32(RADEON_DISP_OUTPUT_CNTL, disp_output_cntl);
		} else {
			dac2_cntl = RREG32(RADEON_DAC_CNTL2) | RADEON_DAC2_DAC_CLK_SEL;
			WREG32(RADEON_DAC_CNTL2, dac2_cntl);
		}
	}

	dac_cntl = (RADEON_DAC_MASK_ALL |
		    RADEON_DAC_VGA_ADR_EN |
		    /* TODO 6-bits */
		    RADEON_DAC_8BIT_EN);

	WREG32_P(RADEON_DAC_CNTL,
		       dac_cntl,
		       RADEON_DAC_RANGE_CNTL |
		       RADEON_DAC_BLANKING);

	if (radeon_encoder->enc_priv) {
		struct radeon_encoder_primary_dac *p_dac = (struct radeon_encoder_primary_dac *)radeon_encoder->enc_priv;
		dac_macro_cntl = p_dac->ps2_pdac_adj;
	} else
		dac_macro_cntl = RREG32(RADEON_DAC_MACRO_CNTL);
	dac_macro_cntl |= RADEON_DAC_PDWN_R | RADEON_DAC_PDWN_G | RADEON_DAC_PDWN_B;
	WREG32(RADEON_DAC_MACRO_CNTL, dac_macro_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
	else
		radeon_combios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
}

static enum drm_connector_status radeon_legacy_primary_dac_detect(struct drm_encoder *encoder,
								  struct drm_connector *connector)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t vclk_ecp_cntl, crtc_ext_cntl;
	uint32_t dac_ext_cntl, dac_cntl, dac_macro_cntl, tmp;
	enum drm_connector_status found = connector_status_disconnected;
	bool color = true;

	/* save the regs we need */
	vclk_ecp_cntl = RREG32_PLL(RADEON_VCLK_ECP_CNTL);
	crtc_ext_cntl = RREG32(RADEON_CRTC_EXT_CNTL);
	dac_ext_cntl = RREG32(RADEON_DAC_EXT_CNTL);
	dac_cntl = RREG32(RADEON_DAC_CNTL);
	dac_macro_cntl = RREG32(RADEON_DAC_MACRO_CNTL);

	tmp = vclk_ecp_cntl &
		~(RADEON_PIXCLK_ALWAYS_ONb | RADEON_PIXCLK_DAC_ALWAYS_ONb);
	WREG32_PLL(RADEON_VCLK_ECP_CNTL, tmp);

	tmp = crtc_ext_cntl | RADEON_CRTC_CRT_ON;
	WREG32(RADEON_CRTC_EXT_CNTL, tmp);

	tmp = RADEON_DAC_FORCE_BLANK_OFF_EN |
		RADEON_DAC_FORCE_DATA_EN;

	if (color)
		tmp |= RADEON_DAC_FORCE_DATA_SEL_RGB;
	else
		tmp |= RADEON_DAC_FORCE_DATA_SEL_G;

	if (ASIC_IS_R300(rdev))
		tmp |= (0x1b6 << RADEON_DAC_FORCE_DATA_SHIFT);
	else
		tmp |= (0x180 << RADEON_DAC_FORCE_DATA_SHIFT);

	WREG32(RADEON_DAC_EXT_CNTL, tmp);

	tmp = dac_cntl & ~(RADEON_DAC_RANGE_CNTL_MASK | RADEON_DAC_PDWN);
	tmp |= RADEON_DAC_RANGE_CNTL_PS2 | RADEON_DAC_CMP_EN;
	WREG32(RADEON_DAC_CNTL, tmp);

	tmp &= ~(RADEON_DAC_PDWN_R |
		 RADEON_DAC_PDWN_G |
		 RADEON_DAC_PDWN_B);

	WREG32(RADEON_DAC_MACRO_CNTL, tmp);

	udelay(2000);

	if (RREG32(RADEON_DAC_CNTL) & RADEON_DAC_CMP_OUTPUT)
		found = connector_status_connected;

	/* restore the regs we used */
	WREG32(RADEON_DAC_CNTL, dac_cntl);
	WREG32(RADEON_DAC_MACRO_CNTL, dac_macro_cntl);
	WREG32(RADEON_DAC_EXT_CNTL, dac_ext_cntl);
	WREG32(RADEON_CRTC_EXT_CNTL, crtc_ext_cntl);
	WREG32_PLL(RADEON_VCLK_ECP_CNTL, vclk_ecp_cntl);

	return found;
}

static const struct drm_encoder_helper_funcs radeon_legacy_primary_dac_helper_funcs = {
	.dpms = radeon_legacy_primary_dac_dpms,
	.mode_fixup = radeon_legacy_primary_dac_mode_fixup,
	.prepare = radeon_legacy_primary_dac_prepare,
	.mode_set = radeon_legacy_primary_dac_mode_set,
	.commit = radeon_legacy_primary_dac_commit,
	.detect = radeon_legacy_primary_dac_detect,
};


static const struct drm_encoder_funcs radeon_legacy_primary_dac_enc_funcs = {
	.destroy = radeon_enc_destroy,
};

static bool radeon_legacy_tmds_int_mode_fixup(struct drm_encoder *encoder,
					      struct drm_display_mode *mode,
					      struct drm_display_mode *adjusted_mode)
{

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	return true;
}

static void radeon_legacy_tmds_int_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t fp_gen_cntl = RREG32(RADEON_FP_GEN_CNTL);
	DRM_DEBUG("\n");

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		fp_gen_cntl |= (RADEON_FP_FPON | RADEON_FP_TMDS_EN);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		fp_gen_cntl &= ~(RADEON_FP_FPON | RADEON_FP_TMDS_EN);
		break;
	}

	WREG32(RADEON_FP_GEN_CNTL, fp_gen_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
	else
		radeon_combios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
}

static void radeon_legacy_tmds_int_prepare(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
	radeon_legacy_tmds_int_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void radeon_legacy_tmds_int_commit(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	radeon_legacy_tmds_int_dpms(encoder, DRM_MODE_DPMS_ON);

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
}

static void radeon_legacy_tmds_int_mode_set(struct drm_encoder *encoder,
					    struct drm_display_mode *mode,
					    struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_crtc *radeon_crtc = to_radeon_crtc(encoder->crtc);
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t tmp, tmds_pll_cntl, tmds_transmitter_cntl, fp_gen_cntl;
	int i;

	DRM_DEBUG("\n");

	if (radeon_crtc->crtc_id == 0)
		radeon_legacy_rmx_mode_set(encoder, mode, adjusted_mode);

	tmp = tmds_pll_cntl = RREG32(RADEON_TMDS_PLL_CNTL);
	tmp &= 0xfffff;
	if (rdev->family == CHIP_RV280) {
		/* bit 22 of TMDS_PLL_CNTL is read-back inverted */
		tmp ^= (1 << 22);
		tmds_pll_cntl ^= (1 << 22);
	}

	if (radeon_encoder->enc_priv) {
		struct radeon_encoder_int_tmds *tmds = (struct radeon_encoder_int_tmds *)radeon_encoder->enc_priv;

		for (i = 0; i < 4; i++) {
			if (tmds->tmds_pll[i].freq == 0)
				break;
			if ((uint32_t)(mode->clock / 10) < tmds->tmds_pll[i].freq) {
				tmp = tmds->tmds_pll[i].value ;
				break;
			}
		}
	}

	if (ASIC_IS_R300(rdev) || (rdev->family == CHIP_RV280)) {
		if (tmp & 0xfff00000)
			tmds_pll_cntl = tmp;
		else {
			tmds_pll_cntl &= 0xfff00000;
			tmds_pll_cntl |= tmp;
		}
	} else
		tmds_pll_cntl = tmp;

	tmds_transmitter_cntl = RREG32(RADEON_TMDS_TRANSMITTER_CNTL) &
		~(RADEON_TMDS_TRANSMITTER_PLLRST);

    if (rdev->family == CHIP_R200 ||
	rdev->family == CHIP_R100 ||
	ASIC_IS_R300(rdev))
	    tmds_transmitter_cntl &= ~(RADEON_TMDS_TRANSMITTER_PLLEN);
    else /* RV chips got this bit reversed */
	    tmds_transmitter_cntl |= RADEON_TMDS_TRANSMITTER_PLLEN;

    fp_gen_cntl = (RREG32(RADEON_FP_GEN_CNTL) |
		   (RADEON_FP_CRTC_DONT_SHADOW_VPAR |
		    RADEON_FP_CRTC_DONT_SHADOW_HEND));

    fp_gen_cntl &= ~(RADEON_FP_FPON | RADEON_FP_TMDS_EN);

    if (1) /*  FIXME rgbBits == 8 */
	    fp_gen_cntl |= RADEON_FP_PANEL_FORMAT;  /* 24 bit format */
    else
	    fp_gen_cntl &= ~RADEON_FP_PANEL_FORMAT;/* 18 bit format */

    if (radeon_crtc->crtc_id == 0) {
	    if (ASIC_IS_R300(rdev) || rdev->family == CHIP_R200) {
		    fp_gen_cntl &= ~R200_FP_SOURCE_SEL_MASK;
		    if (radeon_encoder->flags & RADEON_USE_RMX)
			    fp_gen_cntl |= R200_FP_SOURCE_SEL_RMX;
		    else
			    fp_gen_cntl |= R200_FP_SOURCE_SEL_CRTC1;
	    } else
		    fp_gen_cntl |= RADEON_FP_SEL_CRTC1;
    } else {
	    if (ASIC_IS_R300(rdev) || rdev->family == CHIP_R200) {
		    fp_gen_cntl &= ~R200_FP_SOURCE_SEL_MASK;
		    fp_gen_cntl |= R200_FP_SOURCE_SEL_CRTC2;
	    } else
		    fp_gen_cntl |= RADEON_FP_SEL_CRTC2;
    }

    WREG32(RADEON_TMDS_PLL_CNTL, tmds_pll_cntl);
    WREG32(RADEON_TMDS_TRANSMITTER_CNTL, tmds_transmitter_cntl);
    WREG32(RADEON_FP_GEN_CNTL, fp_gen_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
	else
		radeon_combios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
}

static const struct drm_encoder_helper_funcs radeon_legacy_tmds_int_helper_funcs = {
	.dpms = radeon_legacy_tmds_int_dpms,
	.mode_fixup = radeon_legacy_tmds_int_mode_fixup,
	.prepare = radeon_legacy_tmds_int_prepare,
	.mode_set = radeon_legacy_tmds_int_mode_set,
	.commit = radeon_legacy_tmds_int_commit,
};


static const struct drm_encoder_funcs radeon_legacy_tmds_int_enc_funcs = {
	.destroy = radeon_enc_destroy,
};

static bool radeon_legacy_tmds_ext_mode_fixup(struct drm_encoder *encoder,
					      struct drm_display_mode *mode,
					      struct drm_display_mode *adjusted_mode)
{

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	return true;
}

static void radeon_legacy_tmds_ext_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t fp2_gen_cntl = RREG32(RADEON_FP2_GEN_CNTL);
	DRM_DEBUG("\n");

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		fp2_gen_cntl &= ~RADEON_FP2_BLANK_EN;
		fp2_gen_cntl |= (RADEON_FP2_ON | RADEON_FP2_DVO_EN);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		fp2_gen_cntl |= RADEON_FP2_BLANK_EN;
		fp2_gen_cntl &= ~(RADEON_FP2_ON | RADEON_FP2_DVO_EN);
		break;
	}

	WREG32(RADEON_FP2_GEN_CNTL, fp2_gen_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
	else
		radeon_combios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
}

static void radeon_legacy_tmds_ext_prepare(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
	radeon_legacy_tmds_ext_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void radeon_legacy_tmds_ext_commit(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;
	radeon_legacy_tmds_ext_dpms(encoder, DRM_MODE_DPMS_ON);

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, false);
	else
		radeon_combios_output_lock(encoder, false);
}

static void radeon_legacy_tmds_ext_mode_set(struct drm_encoder *encoder,
					    struct drm_display_mode *mode,
					    struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_crtc *radeon_crtc = to_radeon_crtc(encoder->crtc);
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t fp2_gen_cntl;

	DRM_DEBUG("\n");

	if (radeon_crtc->crtc_id == 0)
		radeon_legacy_rmx_mode_set(encoder, mode, adjusted_mode);

	if (rdev->is_atom_bios) {
		radeon_encoder->pixel_clock = adjusted_mode->clock;
		atombios_external_tmds_setup(encoder, ATOM_ENABLE);
		fp2_gen_cntl = RREG32(RADEON_FP2_GEN_CNTL);
	} else {
		fp2_gen_cntl = RREG32(RADEON_FP2_GEN_CNTL);

		if (1) /*  FIXME rgbBits == 8 */
			fp2_gen_cntl |= RADEON_FP2_PANEL_FORMAT; /* 24 bit format, */
		else
			fp2_gen_cntl &= ~RADEON_FP2_PANEL_FORMAT;/* 18 bit format, */

		fp2_gen_cntl &= ~(RADEON_FP2_ON |
				  RADEON_FP2_DVO_EN |
				  RADEON_FP2_DVO_RATE_SEL_SDR);

		/* XXX: these are oem specific */
		if (ASIC_IS_R300(rdev)) {
			if ((dev->pdev->device == 0x4850) &&
			    (dev->pdev->subsystem_vendor == 0x1028) &&
			    (dev->pdev->subsystem_device == 0x2001)) /* Dell Inspiron 8600 */
				fp2_gen_cntl |= R300_FP2_DVO_CLOCK_MODE_SINGLE;
			else
				fp2_gen_cntl |= RADEON_FP2_PAD_FLOP_EN | R300_FP2_DVO_CLOCK_MODE_SINGLE;

			/*if (mode->clock > 165000)
			  fp2_gen_cntl |= R300_FP2_DVO_DUAL_CHANNEL_EN;*/
		}
	}

	if (radeon_crtc->crtc_id == 0) {
		if ((rdev->family == CHIP_R200) || ASIC_IS_R300(rdev)) {
			fp2_gen_cntl &= ~R200_FP2_SOURCE_SEL_MASK;
			if (radeon_encoder->flags & RADEON_USE_RMX)
				fp2_gen_cntl |= R200_FP2_SOURCE_SEL_RMX;
			else
				fp2_gen_cntl |= R200_FP2_SOURCE_SEL_CRTC1;
		} else
			fp2_gen_cntl &= ~RADEON_FP2_SRC_SEL_CRTC2;
	} else {
		if ((rdev->family == CHIP_R200) || ASIC_IS_R300(rdev)) {
			fp2_gen_cntl &= ~R200_FP2_SOURCE_SEL_MASK;
			fp2_gen_cntl |= R200_FP2_SOURCE_SEL_CRTC2;
		} else
			fp2_gen_cntl |= RADEON_FP2_SRC_SEL_CRTC2;
	}

	WREG32(RADEON_FP2_GEN_CNTL, fp2_gen_cntl);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
	else
		radeon_combios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
}

static const struct drm_encoder_helper_funcs radeon_legacy_tmds_ext_helper_funcs = {
	.dpms = radeon_legacy_tmds_ext_dpms,
	.mode_fixup = radeon_legacy_tmds_ext_mode_fixup,
	.prepare = radeon_legacy_tmds_ext_prepare,
	.mode_set = radeon_legacy_tmds_ext_mode_set,
	.commit = radeon_legacy_tmds_ext_commit,
};


static const struct drm_encoder_funcs radeon_legacy_tmds_ext_enc_funcs = {
	.destroy = radeon_enc_destroy,
};

static bool radeon_legacy_tv_dac_mode_fixup(struct drm_encoder *encoder,
					    struct drm_display_mode *mode,
					    struct drm_display_mode *adjusted_mode)
{

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	return true;
}

static void radeon_legacy_tv_dac_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t fp2_gen_cntl = 0, crtc2_gen_cntl = 0, tv_dac_cntl = 0;
	/* uint32_t tv_master_cntl = 0; */

	DRM_DEBUG("\n");

	if (rdev->family == CHIP_R200)
		fp2_gen_cntl = RREG32(RADEON_FP2_GEN_CNTL);
	else {
		crtc2_gen_cntl = RREG32(RADEON_CRTC2_GEN_CNTL);
		/*  FIXME TV */
		/* tv_master_cntl = RREG32(RADEON_TV_MASTER_CNTL); */
		tv_dac_cntl = RREG32(RADEON_TV_DAC_CNTL);
	}

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (rdev->family == CHIP_R200) {
			fp2_gen_cntl |= (RADEON_FP2_ON | RADEON_FP2_DVO_EN);
		} else {
			crtc2_gen_cntl |= RADEON_CRTC2_CRT2_ON;
			/* tv_master_cntl |= RADEON_TV_ON; */
			if (rdev->family == CHIP_R420 ||
					rdev->family == CHIP_R423 ||
					rdev->family == CHIP_RV410)
				tv_dac_cntl &= ~(R420_TV_DAC_RDACPD |
						R420_TV_DAC_GDACPD |
						R420_TV_DAC_BDACPD |
						RADEON_TV_DAC_BGSLEEP);
			else
				tv_dac_cntl &= ~(RADEON_TV_DAC_RDACPD |
						RADEON_TV_DAC_GDACPD |
						RADEON_TV_DAC_BDACPD |
						RADEON_TV_DAC_BGSLEEP);
		}
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (rdev->family == CHIP_R200)
			fp2_gen_cntl &= ~(RADEON_FP2_ON | RADEON_FP2_DVO_EN);
		else {
			crtc2_gen_cntl &= ~RADEON_CRTC2_CRT2_ON;
			/* tv_master_cntl &= ~RADEON_TV_ON; */
			if (rdev->family == CHIP_R420 ||
					rdev->family == CHIP_R423 ||
					rdev->family == CHIP_RV410)
				tv_dac_cntl |= (R420_TV_DAC_RDACPD |
						R420_TV_DAC_GDACPD |
						R420_TV_DAC_BDACPD |
						RADEON_TV_DAC_BGSLEEP);
			else
				tv_dac_cntl |= (RADEON_TV_DAC_RDACPD |
						RADEON_TV_DAC_GDACPD |
						RADEON_TV_DAC_BDACPD |
						RADEON_TV_DAC_BGSLEEP);
		}
		break;
	}

	if (rdev->family == CHIP_R200) {
		WREG32(RADEON_FP2_GEN_CNTL, fp2_gen_cntl);
	} else {
		WREG32(RADEON_CRTC2_GEN_CNTL, crtc2_gen_cntl);
		/* WREG32(RADEON_TV_MASTER_CNTL, tv_master_cntl); */
		WREG32(RADEON_TV_DAC_CNTL, tv_dac_cntl);
	}

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
	else
		radeon_combios_encoder_dpms_scratch_regs(encoder, (mode == DRM_MODE_DPMS_ON) ? true : false);
}

static void radeon_legacy_tv_dac_prepare(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
	radeon_legacy_tv_dac_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void radeon_legacy_tv_dac_commit(struct drm_encoder *encoder)
{
	struct radeon_device *rdev = encoder->dev->dev_private;

	radeon_legacy_tv_dac_dpms(encoder, DRM_MODE_DPMS_ON);

	if (rdev->is_atom_bios)
		radeon_atom_output_lock(encoder, true);
	else
		radeon_combios_output_lock(encoder, true);
}

static void radeon_legacy_tv_dac_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_crtc *radeon_crtc = to_radeon_crtc(encoder->crtc);
	struct radeon_encoder *radeon_encoder = to_radeon_encoder(encoder);
	uint32_t tv_dac_cntl, gpiopad_a = 0, dac2_cntl, disp_output_cntl = 0;
	uint32_t disp_hw_debug = 0, fp2_gen_cntl = 0;

	DRM_DEBUG("\n");

	if (radeon_crtc->crtc_id == 0)
		radeon_legacy_rmx_mode_set(encoder, mode, adjusted_mode);

	if (rdev->family != CHIP_R200) {
		tv_dac_cntl = RREG32(RADEON_TV_DAC_CNTL);
		if (rdev->family == CHIP_R420 ||
				rdev->family == CHIP_R423 ||
				rdev->family == CHIP_RV410) {
			tv_dac_cntl &= ~(RADEON_TV_DAC_STD_MASK |
					RADEON_TV_DAC_BGADJ_MASK |
					R420_TV_DAC_DACADJ_MASK |
					R420_TV_DAC_RDACPD |
					R420_TV_DAC_GDACPD |
					R420_TV_DAC_GDACPD |
					R420_TV_DAC_TVENABLE);
		} else {
			tv_dac_cntl &= ~(RADEON_TV_DAC_STD_MASK |
					RADEON_TV_DAC_BGADJ_MASK |
					RADEON_TV_DAC_DACADJ_MASK |
					RADEON_TV_DAC_RDACPD |
					RADEON_TV_DAC_GDACPD |
					RADEON_TV_DAC_GDACPD);
		}

		/*  FIXME TV */
		if (radeon_encoder->enc_priv) {
			struct radeon_encoder_tv_dac *tv_dac = radeon_encoder->enc_priv;
			tv_dac_cntl |= (RADEON_TV_DAC_NBLANK |
					RADEON_TV_DAC_NHOLD |
					RADEON_TV_DAC_STD_PS2 |
					tv_dac->ps2_tvdac_adj);
		} else
			tv_dac_cntl |= (RADEON_TV_DAC_NBLANK |
					RADEON_TV_DAC_NHOLD |
					RADEON_TV_DAC_STD_PS2);

		WREG32(RADEON_TV_DAC_CNTL, tv_dac_cntl);
	}

	if (ASIC_IS_R300(rdev)) {
		gpiopad_a = RREG32(RADEON_GPIOPAD_A) | 1;
		disp_output_cntl = RREG32(RADEON_DISP_OUTPUT_CNTL);
	} else if (rdev->family == CHIP_R200)
		fp2_gen_cntl = RREG32(RADEON_FP2_GEN_CNTL);
	else
		disp_hw_debug = RREG32(RADEON_DISP_HW_DEBUG);

	dac2_cntl = RREG32(RADEON_DAC_CNTL2) | RADEON_DAC2_DAC2_CLK_SEL;

	if (radeon_crtc->crtc_id == 0) {
		if (ASIC_IS_R300(rdev)) {
			disp_output_cntl &= ~RADEON_DISP_TVDAC_SOURCE_MASK;
			disp_output_cntl |= RADEON_DISP_TVDAC_SOURCE_CRTC;
		} else if (rdev->family == CHIP_R200) {
			fp2_gen_cntl &= ~(R200_FP2_SOURCE_SEL_MASK |
					  RADEON_FP2_DVO_RATE_SEL_SDR);
		} else
			disp_hw_debug |= RADEON_CRT2_DISP1_SEL;
	} else {
		if (ASIC_IS_R300(rdev)) {
			disp_output_cntl &= ~RADEON_DISP_TVDAC_SOURCE_MASK;
			disp_output_cntl |= RADEON_DISP_TVDAC_SOURCE_CRTC2;
		} else if (rdev->family == CHIP_R200) {
			fp2_gen_cntl &= ~(R200_FP2_SOURCE_SEL_MASK |
					  RADEON_FP2_DVO_RATE_SEL_SDR);
			fp2_gen_cntl |= R200_FP2_SOURCE_SEL_CRTC2;
		} else
			disp_hw_debug &= ~RADEON_CRT2_DISP1_SEL;
	}

	WREG32(RADEON_DAC_CNTL2, dac2_cntl);

	if (ASIC_IS_R300(rdev)) {
		WREG32_P(RADEON_GPIOPAD_A, gpiopad_a, ~1);
		WREG32(RADEON_DISP_TV_OUT_CNTL, disp_output_cntl);
	} else if (rdev->family == CHIP_R200)
		WREG32(RADEON_FP2_GEN_CNTL, fp2_gen_cntl);
	else
		WREG32(RADEON_DISP_HW_DEBUG, disp_hw_debug);

	if (rdev->is_atom_bios)
		radeon_atombios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);
	else
		radeon_combios_encoder_crtc_scratch_regs(encoder, radeon_crtc->crtc_id);

}

static enum drm_connector_status radeon_legacy_tv_dac_detect(struct drm_encoder *encoder,
							     struct drm_connector *connector)
{
	struct drm_device *dev = encoder->dev;
	struct radeon_device *rdev = dev->dev_private;
	uint32_t crtc2_gen_cntl, tv_dac_cntl, dac_cntl2, dac_ext_cntl;
	uint32_t disp_hw_debug, disp_output_cntl, gpiopad_a, pixclks_cntl, tmp;
	enum drm_connector_status found = connector_status_disconnected;
	bool color = true;

	/*  FIXME tv */

	/* save the regs we need */
	pixclks_cntl = RREG32_PLL(RADEON_PIXCLKS_CNTL);
	gpiopad_a = ASIC_IS_R300(rdev) ? RREG32(RADEON_GPIOPAD_A) : 0;
	disp_output_cntl = ASIC_IS_R300(rdev) ? RREG32(RADEON_DISP_OUTPUT_CNTL) : 0;
	disp_hw_debug = ASIC_IS_R300(rdev) ? 0 : RREG32(RADEON_DISP_HW_DEBUG);
	crtc2_gen_cntl = RREG32(RADEON_CRTC2_GEN_CNTL);
	tv_dac_cntl = RREG32(RADEON_TV_DAC_CNTL);
	dac_ext_cntl = RREG32(RADEON_DAC_EXT_CNTL);
	dac_cntl2 = RREG32(RADEON_DAC_CNTL2);

	tmp = pixclks_cntl & ~(RADEON_PIX2CLK_ALWAYS_ONb
			       | RADEON_PIX2CLK_DAC_ALWAYS_ONb);
	WREG32_PLL(RADEON_PIXCLKS_CNTL, tmp);

	if (ASIC_IS_R300(rdev))
		WREG32_P(RADEON_GPIOPAD_A, 1, ~1);

	tmp = crtc2_gen_cntl & ~RADEON_CRTC2_PIX_WIDTH_MASK;
	tmp |= RADEON_CRTC2_CRT2_ON |
		(2 << RADEON_CRTC2_PIX_WIDTH_SHIFT);

	WREG32(RADEON_CRTC2_GEN_CNTL, tmp);

	if (ASIC_IS_R300(rdev)) {
		tmp = disp_output_cntl & ~RADEON_DISP_TVDAC_SOURCE_MASK;
		tmp |= RADEON_DISP_TVDAC_SOURCE_CRTC2;
		WREG32(RADEON_DISP_OUTPUT_CNTL, tmp);
	} else {
		tmp = disp_hw_debug & ~RADEON_CRT2_DISP1_SEL;
		WREG32(RADEON_DISP_HW_DEBUG, tmp);
	}

	tmp = RADEON_TV_DAC_NBLANK |
		RADEON_TV_DAC_NHOLD |
		RADEON_TV_MONITOR_DETECT_EN |
		RADEON_TV_DAC_STD_PS2;

	WREG32(RADEON_TV_DAC_CNTL, tmp);

	tmp = RADEON_DAC2_FORCE_BLANK_OFF_EN |
		RADEON_DAC2_FORCE_DATA_EN;

	if (color)
		tmp |= RADEON_DAC_FORCE_DATA_SEL_RGB;
	else
		tmp |= RADEON_DAC_FORCE_DATA_SEL_G;

	if (ASIC_IS_R300(rdev))
		tmp |= (0x1b6 << RADEON_DAC_FORCE_DATA_SHIFT);
	else
		tmp |= (0x180 << RADEON_DAC_FORCE_DATA_SHIFT);

	WREG32(RADEON_DAC_EXT_CNTL, tmp);

	tmp = dac_cntl2 | RADEON_DAC2_DAC2_CLK_SEL | RADEON_DAC2_CMP_EN;
	WREG32(RADEON_DAC_CNTL2, tmp);

	udelay(10000);

	if (ASIC_IS_R300(rdev)) {
		if (RREG32(RADEON_DAC_CNTL2) & RADEON_DAC2_CMP_OUT_B)
			found = connector_status_connected;
	} else {
		if (RREG32(RADEON_DAC_CNTL2) & RADEON_DAC2_CMP_OUTPUT)
			found = connector_status_connected;
	}

	/* restore regs we used */
	WREG32(RADEON_DAC_CNTL2, dac_cntl2);
	WREG32(RADEON_DAC_EXT_CNTL, dac_ext_cntl);
	WREG32(RADEON_TV_DAC_CNTL, tv_dac_cntl);
	WREG32(RADEON_CRTC2_GEN_CNTL, crtc2_gen_cntl);

	if (ASIC_IS_R300(rdev)) {
		WREG32(RADEON_DISP_OUTPUT_CNTL, disp_output_cntl);
		WREG32_P(RADEON_GPIOPAD_A, gpiopad_a, ~1);
	} else {
		WREG32(RADEON_DISP_HW_DEBUG, disp_hw_debug);
	}
	WREG32_PLL(RADEON_PIXCLKS_CNTL, pixclks_cntl);

	/* return found; */
	return connector_status_disconnected;

}

static const struct drm_encoder_helper_funcs radeon_legacy_tv_dac_helper_funcs = {
	.dpms = radeon_legacy_tv_dac_dpms,
	.mode_fixup = radeon_legacy_tv_dac_mode_fixup,
	.prepare = radeon_legacy_tv_dac_prepare,
	.mode_set = radeon_legacy_tv_dac_mode_set,
	.commit = radeon_legacy_tv_dac_commit,
	.detect = radeon_legacy_tv_dac_detect,
};


static const struct drm_encoder_funcs radeon_legacy_tv_dac_enc_funcs = {
	.destroy = radeon_enc_destroy,
};

void
radeon_add_legacy_encoder(struct drm_device *dev, uint32_t encoder_id, uint32_t supported_device)
{
	struct radeon_device *rdev = dev->dev_private;
	struct drm_encoder *encoder;
	struct radeon_encoder *radeon_encoder;

	/* see if we already added it */
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		radeon_encoder = to_radeon_encoder(encoder);
		if (radeon_encoder->encoder_id == encoder_id) {
			radeon_encoder->devices |= supported_device;
			return;
		}

	}

	/* add a new one */
	radeon_encoder = kzalloc(sizeof(struct radeon_encoder), GFP_KERNEL);
	if (!radeon_encoder)
		return;

	encoder = &radeon_encoder->base;
	encoder->possible_crtcs = 0x3;
	encoder->possible_clones = 0;

	radeon_encoder->enc_priv = NULL;

	radeon_encoder->encoder_id = encoder_id;
	radeon_encoder->devices = supported_device;

	switch (radeon_encoder->encoder_id) {
	case ENCODER_OBJECT_ID_INTERNAL_LVDS:
		drm_encoder_init(dev, encoder, &radeon_legacy_lvds_enc_funcs, DRM_MODE_ENCODER_LVDS);
		drm_encoder_helper_add(encoder, &radeon_legacy_lvds_helper_funcs);
		if (rdev->is_atom_bios)
			radeon_encoder->enc_priv = radeon_atombios_get_lvds_info(radeon_encoder);
		else
			radeon_encoder->enc_priv = radeon_combios_get_lvds_info(radeon_encoder);
		radeon_encoder->rmx_type = RMX_FULL;
		break;
	case ENCODER_OBJECT_ID_INTERNAL_TMDS1:
		drm_encoder_init(dev, encoder, &radeon_legacy_tmds_int_enc_funcs, DRM_MODE_ENCODER_TMDS);
		drm_encoder_helper_add(encoder, &radeon_legacy_tmds_int_helper_funcs);
		if (rdev->is_atom_bios)
			radeon_encoder->enc_priv = radeon_atombios_get_tmds_info(radeon_encoder);
		else
			radeon_encoder->enc_priv = radeon_combios_get_tmds_info(radeon_encoder);
		break;
	case ENCODER_OBJECT_ID_INTERNAL_DAC1:
		drm_encoder_init(dev, encoder, &radeon_legacy_primary_dac_enc_funcs, DRM_MODE_ENCODER_DAC);
		drm_encoder_helper_add(encoder, &radeon_legacy_primary_dac_helper_funcs);
		if (rdev->is_atom_bios)
			radeon_encoder->enc_priv = radeon_atombios_get_primary_dac_info(radeon_encoder);
		else
			radeon_encoder->enc_priv = radeon_combios_get_primary_dac_info(radeon_encoder);
		break;
	case ENCODER_OBJECT_ID_INTERNAL_DAC2:
		drm_encoder_init(dev, encoder, &radeon_legacy_tv_dac_enc_funcs, DRM_MODE_ENCODER_TVDAC);
		drm_encoder_helper_add(encoder, &radeon_legacy_tv_dac_helper_funcs);
		if (rdev->is_atom_bios)
			radeon_encoder->enc_priv = radeon_atombios_get_tv_dac_info(radeon_encoder);
		else
			radeon_encoder->enc_priv = radeon_combios_get_tv_dac_info(radeon_encoder);
		break;
	case ENCODER_OBJECT_ID_INTERNAL_DVO1:
		drm_encoder_init(dev, encoder, &radeon_legacy_tmds_ext_enc_funcs, DRM_MODE_ENCODER_TMDS);
		drm_encoder_helper_add(encoder, &radeon_legacy_tmds_ext_helper_funcs);
		if (!rdev->is_atom_bios)
			radeon_combios_get_ext_tmds_info(radeon_encoder);
		break;
	}
}
