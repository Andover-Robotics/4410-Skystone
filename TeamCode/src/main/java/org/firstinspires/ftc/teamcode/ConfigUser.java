package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.config.Configuration;

import java.io.IOException;

public class ConfigUser<SCHEMA_TYPE> {
  public SCHEMA_TYPE config;

  public ConfigUser(String configFilename, SCHEMA_TYPE newInstance) {
    try {

      config = Configuration.fromPropertiesFile(configFilename).loadToSchema(newInstance);

    } catch (IOException io) {
      throw new RuntimeException(io);
    }
  }
}
