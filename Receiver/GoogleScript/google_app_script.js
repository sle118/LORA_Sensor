/**
 * Google Apps Script for Data Logging to Google Sheets
 *
 * Overview:
 * This script is designed for seamless data logging from IoT devices and other sources capable of sending HTTP GET requests.
 * It appends received data to specified Google Sheets, managing dynamic parameter inclusion and ensuring robust data logging.
 *
 * Key Features:
 * - **doGet(e)**: Handles GET requests by appending data to a Google Sheet. It formats the sheet for new entries
 *   with headers and enables filtering. Sheets are named dynamically based on UUID or default to "data".
 * - **doPost(e)**: Reserved for future implementations to handle POST requests with JSON payloads.
 * - **findOrCreateColumn(headerName, sheet)**: Dynamically manages sheet columns to accommodate new data fields, ensuring no data is misplaced.
 * - **logMessage(spreadsheet, message)**: Logs operations and errors in a 'logs' sheet within the same spreadsheet and via the Logger service.
 * - **lookupValue(key, rawValue, spreadsheet)**: Fetches human-readable values for specified keys using the 'lookups' sheet, enhancing clarity of logged data.
 * - **getFilteredData(parameters, ignoredValues, spreadsheet)**: Filters and processes parameters to exclude designated keys and enhance data with human-readable equivalents.
 *
 * Usage:
 * Deploy this script as a Web App associated with a Google Sheet to serve as a data endpoint for HTTP GET requests.
 * Ensure that the script has appropriate permissions to access the Google Sheets API and that the Google Sheet is shared correctly.
 *
 * Example GET request format:
 * ```
 * https://script.google.com/macros/s/your_script_id/exec?temperature=22.5&humidity=48
 * ```
 *
 * Deployment:
 * 1. Open the Google Apps Script editor via your Google Sheet.
 * 2. Paste this script into the editor.
 * 3. Deploy as a Web App:
 *    - Set 'Execute as' to your account.
 *    - Set 'Who has access' to 'Anyone with the link' or 'Anyone'.
 * 4. Note the generated URL of the deployed Web App; this is your endpoint for HTTP GET requests.
 *
 * Error Handling:
 * Logs errors and operational messages to the 'logs' sheet and optionally sends email notifications for critical issues.
 * 
 * Dependencies:
 * This script uses the SpreadsheetApp and MailApp services from Google Apps Script. Be mindful of Google's quota limits for URL Fetch calls and script execution times.
 *
 * MIT License:
 * This script is released under the MIT license, which permits reuse, modification, and distribution for both private and commercial purposes.
 */



var ignoredValues = ['sheet_name', 'command'];
function test() {
  doGet({ "contentLength": -1, "parameter": { "temperature": "25.08788681", "rawVbat": "0", "internal_voltage": "0", "command": "append_row", "sheet_name": "distance", "packetnum": "10087", "duration": "0", "bat_voltage": "0.019335937", "core_voltage": "0", "uuid": "04:56:84:ba:04:77:c2:03", "internal_temp": "0", "distance": "0", "snr": "8" }, "parameters": { "snr": ["8"], "uuid": ["04:56:84:ba:04:77:c2:03"], "core_voltage": ["0"], "internal_voltage": ["0"], "temperature": ["25.08788681"], "duration": ["0"], "internal_temp": ["0"], "distance": ["0"], "sheet_name": ["distance"], "bat_voltage": ["0.019335937"], "command": ["append_row"], "packetnum": ["10087"], "rawVbat": ["0"] }, "queryString": "sheet_name=distance&bat_voltage=0%2e019335937&rawVbat=0&distance=0&duration=0&temperature=25%2e08788681&internal_temp=0&internal_voltage=0&core_voltage=0&packetnum=10087&snr=8&uuid=04%3a56%3a84%3aba%3a04%3a77%3ac2%3a03&command=append%5frow", "contextPath": "" });
}
/**
 * Handles GET requests by appending data from URL parameters to a Google Sheet based on UUID or a default sheet.
 * The function logs actions and optionally sends email notifications upon errors.
 * @param {Object} e Event object that contains the URL parameters.
 * @returns {GoogleAppsScript.Content.TextOutput} JSON output indicating success or failure.
 */
function doGet(e) {
  var result = {
    status: 'success',
    message: 'Data received via GET',
    command: e.parameter.command || "append_row"
  };

  try {
    var rowValues = [];
    var sheetDoc = e.parameter.document || SpreadsheetApp.getActiveSpreadsheet().getId();
    var SS = SpreadsheetApp.openById(sheetDoc);
    var filteredData = getFilteredData(e.parameters, ignoredValues, SS);

    // Determine sheet name based on uuid lookup
    var uuidLookup = filteredData.uuid ? lookupValueRaw('uuid', e.parameter.uuid, SS) : null;
    var sheetName = e.parameter.sheet_name || "data";
    sheetName = uuidLookup ? `${sheetName}_${uuidLookup}` : sheetName;

    var sheet = SS.getSheetByName(sheetName) || SS.insertSheet(sheetName);

    if (result.command == "append_row") {
      if (sheet.getLastRow() === 0) {
        // Append header row with keys from filteredData
        sheet.appendRow(Object.keys(filteredData));

        // Freeze the first row
        sheet.setFrozenRows(1);

        // Auto resize columns
        sheet.autoResizeColumns(1, sheet.getLastColumn());

        // Set filter on the first row
        sheet.getRange(1, 1, 1, sheet.getLastColumn()).createFilter();
      }

      Object.keys(filteredData).forEach(key => {
        var columnIndex = findOrCreateColumn(key, sheet);
        // Place data into the array based on columnIndex
        rowValues[columnIndex - 1] = filteredData[key]; // -1 because array is 0-based
      });

      // Ensure all indices up to max column index are initialized
      var maxIndex = sheet.getLastColumn();
      for (var i = 0; i < maxIndex; i++) {
        if (rowValues[i] === undefined) {
          rowValues[i] = ""; // Set default empty value for undefined indices
        }
      }

      sheet.appendRow(rowValues);
      SpreadsheetApp.flush(); // Ensure changes are written immediately

      result.message = `OK ${sheetName} #${sheet.getLastRow()-1}`;
    }
    else {
      result.status = 'Error';
      result.message = 'Unsupported command (' + result.command + ')';
    }
  } catch (error) {
    result.status = 'failed';
    result.message = 'Error processing GET data: ' + error.toString();
    MailApp.sendEmail({ to: 'sle118@hotmail.com', subject: "Json-To-Sheet GET ERROR", body: {"error":error.toString(),"context":e} });
  }

  logMessage(SS, result.message);
  return ContentService.createTextOutput(JSON.stringify(result)).setMimeType(ContentService.MimeType.JSON);
}



function doPost(e) {
  var result = {
    status: 'error',
    message: 'Unimplemented method POST'
  };

  // MailApp.sendEmail({to:'sle118@hotmail.com',subject: "Json-To-Sheet POST Execution log",body: Logger.getLog()});
  return ContentService.createTextOutput(result).setMimeType(ContentService.MimeType.JSON);
}

/**
 * Finds or creates a column for a given header name.
 * @param {String} headerName - The name of the header/column.
 * @param {Sheet} sheet - The sheet object where the header is to be found/added.
 * @return {Number} The index of the column.
 */
function findOrCreateColumn(headerName, sheet) {
  const headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  let columnIndex = headers.indexOf(headerName.toLowerCase());
  if (columnIndex === -1) {
    columnIndex = headers.indexOf(headerName);
  }
  if (columnIndex === -1) {
    columnIndex = headers.length;
    sheet.getRange(1, columnIndex + 1).setValue(headerName);
  }
  return columnIndex + 1; // Adding 1 because spreadsheet columns are 1-based
}

/**
 * Logs a message to the "logs" sheet in the provided spreadsheet and to the Logger service.
 * @param {Spreadsheet} spreadsheet - The spreadsheet object where logs are to be recorded.
 * @param {String} message - The message to log.
 */
function logMessage(spreadsheet, message) {
  const logSheetName = 'logs';
  let logSheet = spreadsheet.getSheetByName(logSheetName);

  // Create the "logs" sheet if it does not exist
  if (!logSheet) {
    logSheet = spreadsheet.insertSheet(logSheetName);
    logSheet.appendRow(['Timestamp', 'Message']); // Add headers
  }

  // Get the current date and time
  const timestamp = Utilities.formatDate(new Date(), Session.getScriptTimeZone(), "yyyy/MM/dd HH:mm:ss");

  // Append the log message
  logSheet.appendRow([timestamp, message]);

  // Log to the Logger service
  Logger.log(message);
}

/**
 * Retrieves a human-readable value for a given key from the 'lookups' sheet without formatting.
 * @param {String} key - The parameter name for which the lookup is done.
 * @param {String} rawValue - The raw value to look up.
 * @param {Spreadsheet} spreadsheet - The spreadsheet object where the 'lookups' sheet resides.
 * @return {String} - The human-readable value or the raw value if no lookup is found.
 */
function lookupValueRaw(key, rawValue, spreadsheet) {
  const lookupsSheet = spreadsheet.getSheetByName('lookups');
  if (!lookupsSheet) return rawValue; // Return raw value if no 'lookups' sheet exists

  const lookups = lookupsSheet.getRange(2, 1, lookupsSheet.getLastRow(), 3).getValues();
  for (const [variable, from, to] of lookups) {
    if (variable === key && from === rawValue) {
      return to; // Return only the human-readable name
    }
  }
  return rawValue; // Return raw value if no match is found
}
/**
 * Retrieves and formats a human-readable value for a given key from the 'lookups' sheet.
 * Uses lookupValueRaw to fetch the value and formats it for display.
 * @param {String} key - The parameter name for which the lookup is done.
 * @param {String} rawValue - The raw value to look up.
 * @param {Spreadsheet} spreadsheet - The spreadsheet object where the 'lookups' sheet resides.
 * @return {String} - The formatted human-readable value or the raw value if no lookup is found.
 */
function lookupValue(key, rawValue, spreadsheet) {
  const humanReadable = lookupValueRaw(key, rawValue, spreadsheet);
  return humanReadable === rawValue ? rawValue : `${humanReadable} (${rawValue})`;
}


/**
 * Filters the parameters to remove ignored values and replace values with lookups.
 * @param {Object} parameters - The parameters object containing data from the request.
 * @param {Array} ignoredValues - An array of parameter keys to ignore.
 * @param {Spreadsheet} spreadsheet - The spreadsheet object used for lookups.
 * @return {Object} - The filtered and augmented data object.
 */
function getFilteredData(parameters, ignoredValues, spreadsheet) {
  let filteredData = { timestamp: Utilities.formatDate(new Date(), Session.getScriptTimeZone(), "yyyy/MM/dd HH:mm:ss") };
  Object.keys(parameters).forEach(key => {
    if (!ignoredValues.includes(key)) {
      const rawValue = parameters[key][0];
      filteredData[key] = lookupValue(key, rawValue, spreadsheet); // Replace or keep raw value
    }
  });
  return filteredData;
}


