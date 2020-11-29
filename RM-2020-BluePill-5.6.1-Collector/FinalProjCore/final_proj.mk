FINAL_PROJ_CORE_DIR = FinalProjCore

CPP_SRC += \
$(wildcard $(FINAL_PROJ_CORE_DIR)/Drivers/*.cpp) \
$(wildcard $(FINAL_PROJ_CORE_DIR)/Utility/*.cpp) \
$(wildcard $(FINAL_PROJ_CORE_DIR)/Control/*.cpp) \
$(wildcard $(FINAL_PROJ_CORE_DIR)/Communication/*.cpp) 


INC += \
-I$(FINAL_PROJ_CORE_DIR)/Drivers \
-I$(FINAL_PROJ_CORE_DIR)/Utility \
-I$(FINAL_PROJ_CORE_DIR)/Control \
-I$(FINAL_PROJ_CORE_DIR)/Communication

DEFS += \
-DTRUE=1 \
-DNULL=0
