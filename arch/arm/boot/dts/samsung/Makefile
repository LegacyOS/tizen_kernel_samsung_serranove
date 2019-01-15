ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_SEC_KIRANLTE_PROJECT) += msm8916-tizen_kiranlte-r00.dtb \
					msm8916-tizen_kiranlte-r01.dtb

dtb-$(CONFIG_SEC_GRANDPRIME_PROJECT) += msm8916-tizen_grandprime-r01.dtb\
					msm8916-tizen_grandprime-r02.dtb\
					msm8916-tizen_grandprime-r03.dtb

dtb-$(CONFIG_SEC_J5LTE_PROJECT) += msm8916-tizen_j5lte-r05.dtb

dtb-$(CONFIG_SEC_Z3LTE_CIS_PROJECT) += msm8916-tizen_z3lte_cis-r00.dtb\
					msm8916-tizen_z3lte_cis-r01.dtb

dtb-$(CONFIG_SEC_Z3LTE_CHN_PROJECT) += msm8916-tizen_z3lte_chn-r00.dtb\
					msm8916-tizen_z3lte_chn-r01.dtb

dtb-$(CONFIG_SEC_MILLET_PROJECT) += msm8916-tizen_millet-r00.dtb

dtb-$(CONFIG_SEC_HIVE_PROJECT) += msm8916-tizen_hive-r00.dtb \
					msm8916-tizen_hive-r01.dtb \
					msm8916-tizen_hive-r02.dtb \
					msm8916-tizen_hive-r03.dtb \
					msm8916-tizen_hive-r04.dtb \
					msm8916-tizen_hive-r05.dtb \
					msm8916-tizen_hive-r06.dtb \
					msm8916-tizen_hive-r07.dtb \
					msm8916-tizen_hive-r08.dtb \
					msm8916-tizen_hive-r09.dtb \
					msm8916-tizen_hive-r10.dtb

dtb-$(CONFIG_SEC_M2M_PROJECT) += msm8916-tizen_m2m-r00.dtb \
				msm8916-tizen_m2m-r01.dtb

dtb-$(CONFIG_SEC_KITT_PROJECT) += msm8916-tizen_kitt-r00.dtb

dtb-$(CONFIG_SEC_TETRA_PROJECT) += msm8916-tizen_tetra-r00.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
endif

$(obj)/../%.dtb: $(src)/%.dts
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
