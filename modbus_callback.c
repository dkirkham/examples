int
acmb_write(modbus_t * ctx, int function, uint16_t address, int nb,
           const uint8_t * req_buf, int size, modbus_mapping_t * mb_mapping)
{
  int rc = 0;
  int unit = address / ACD_UNIT_INCREMENT;
  int unit_address = unit * ACD_UNIT_INCREMENT;
  uint16_t value;
  int i;

  /* First, check the bounds on the address and nb */
  if (unit >= ACD_UNITS)
    return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

  /* normalise address per AC unit */
  address -= unit_address;

  switch (function)
    {
    case _FC_WRITE_SINGLE_REGISTER:
    case _FC_WRITE_MULTIPLE_REGISTERS:
    case _FC_WRITE_AND_READ_REGISTERS:
      /* Check bounds and values first, before committing any values to the device state */
      for (i = address; i < address + nb; i++)
        {
          rc = modbus_request_get_register(req_buf, size, address + unit_address, i, &value);
          if (rc != 0)
            return rc;

          switch (i)
            {
            case 0:
              if (value > 4 || value < 0)
                return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
              break;
            case 1:
              if (value > 14 || value < 0)
                return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
              break;
            case 2:
              if (value > 34 || value < 0)
                return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
              break;
            case 3:
              if (value > 3 || value < 0)
                return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
              break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
        }

      /* Now update the device state */
      for (i = address; i < address + nb; i++)
        {
          rc = modbus_request_get_register(req_buf, size, address + unit_address, i, &value);
          if (rc != 0)
            return rc;

          switch (i)
            {
            case 0: acst[unit].mode = value; break;
            case 1: acst[unit].temp = value; break;
            case 2: acst[unit].timer = value; break;
            case 3: acst[unit].fan = value; break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
        }

      /* TODO: send commands to the device to activate state change… */
      break;
    case _FC_WRITE_SINGLE_COIL:
      rc = modbus_request_get_register(req_buf, size, address + unit_address, 0, &value);
      if (rc != 0)
        return rc;

      if (value == 0xFF00)
        value = 1;
      else if (value == 0x0)
        value = 0;
      else
        return MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;

      switch (address)
        {
        case 0: acst[unit].power = value; break;
        case 1: acst[unit].swing = value; break;
        default:
          return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }

      /* TODO: send commands to the device to activate state change… */
      break;
    case _FC_WRITE_MULTIPLE_COILS:
      for (i = address; i < address + nb; i++)
        {
          rc = modbus_request_get_bit(req_buf, size, address + unit_address, i, &value);
          if (rc != 0)
            return rc;

          switch (i)
            {
            case 0: acst[unit].power = value; break;
            case 1: acst[unit].swing = value; break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
        }

      /* TODO: send commands to the device to activate state change… */
      break;
    default:
      if (debug)
        {
          fprintf(stderr, "Internal error: write access called with wrong function code %d\n", function);
        }
      return MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
    }
  return 0;
}

int
acmb_read(modbus_t * ctx, int function, uint16_t address, int nb,
          uint8_t * rsp_buf, int *size, const modbus_mapping_t * mb_mapping)
{
  int rc = 0;
  int unit = address / ACD_UNIT_INCREMENT;
  int unit_address = unit * ACD_UNIT_INCREMENT;
  uint16_t value;
  int i;

  /* First, check the bounds on the address and nb */
  if (unit >= ACD_UNITS)
    return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

  /* normalise address per AC unit */
  address -= unit_address;

  switch (function)
    {
    case _FC_READ_HOLDING_REGISTERS:
    case _FC_WRITE_AND_READ_REGISTERS:
      for (i = address; i < address + nb; i++)
        {
          switch (i)
            {
            case 0:
              value = acst[unit].mode;
              break;
            case 1:
              value = acst[unit].temp;
              break;
            case 2:
              value = acst[unit].timer;
              break;
            case 3:
              value = acst[unit].fan;
              break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
          rc = modbus_response_set_register(rsp_buf, size, address + unit_address, i, value);
          if (rc != 0)
            return rc;
        }
      break;
    case _FC_READ_COILS:
      for (i = address; i < address + nb; i++)
        {
          switch (i)
            {
            case 0:
              value = acst[unit].power;
              break;
            case 1:
              value = acst[unit].swing;
              break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
          rc = modbus_response_set_bit(rsp_buf, size, address + unit_address, i, value);
          if (rc != 0)
            return rc;
        }
      break;
    case _FC_READ_DISCRETE_INPUTS:
      for (i = address; i < address + nb; i++)
        {
          switch (i)
            {
            case 0:
              value = acst[unit].powered;
              break;
            case 1:
              value = acst[unit].comprunning;
              break;
            case 2:
              value = acst[unit].timerrunning;
              break;
            default:
              return MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            }
          rc = modbus_response_set_bit(rsp_buf, size, address + unit_address, i, value);
          if (rc != 0)
            return rc;
        }
      break;
   default:
      if (debug)
        {
          fprintf(stderr, "Internal error: read access called with wrong function code %d\n", function);
        }
      return MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
    }

  return 0;
}


