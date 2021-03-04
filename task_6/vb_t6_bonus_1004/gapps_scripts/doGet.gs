function doGet(e){
  
  var ss = SpreadsheetApp.getActive();
  
  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();
  var lastCol = sheet.getLastColumn();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  var rowOffset = -1;

  var orderIdCol = 0;
  var dispatchTimeCol = 0;

  for (i in headers){
    if (headers[i] == "Order ID") {
      orderIdCol = i;
    }
    if (headers[i] == "Order Time") {
      orderTimeCol = i;
    }
  }

  if(e.parameter["id"] == "Dashboard"){
    var range = sheet.getDataRange();
    var values = range.getValues();
    console.log(values.length)

    for(i = 1; i<values.length; i++){
      if(values[i][orderIdCol] == e.parameter["Order ID"]){
        rowOffset = i;
        Logger.log(rowOffset);
        break;
      }
    }
  }
  
  for (i in headers){
    
    if (headers[i] == "Timestamp"){
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else{
      val = e.parameter[headers[i]]; 
    }

    if(e.parameter["id"]=="Dashboard" && rowOffset!=-1){
      if(cell.offset(rowOffset, col).isBlank()) {
        cell.offset(rowOffset,col).setValue(val);
      }
      if(headers[i] == "Shipping Time" && !cell.offset(rowOffset, col).isBlank()) {
        var orderTime = cell.offset(rowOffset,orderTimeCol).getValue()
        var shipTime = cell.offset(rowOffset,col).getValue()
        var delta = (shipTime - orderTime)/1000
        console.log(delta, shipTime, orderTime, col)
        cell.offset(rowOffset, lastCol-1).setValue(delta)
      }
    }
    
    else{
    cell.offset(lastRow, col).setValue(val);
    }
    col++;
  }

  if(e.parameter["id"] == "OrdersShipped"){
    var to = "eyrc.vb.1004@gmail.com";  
    var toins = "eyrc.vb.0000@gmail.com";
    var message=`
Hello! Your Order has been shipped! It will be delivered to you by ${e.parameter["Estimated Time of Delivery"]}.
Contact us incase you have any questions, we are here to help you :)

ORDER SUMMARY:

Order Number: ${e.parameter["Order ID"]}
Item: ${e.parameter["Item"]}
Quantity: ${e.parameter["Shipped Quantity"]}
Shipped Date and Time: ${e.parameter["Shipped Date and Time"]}
City: ${e.parameter["City"]}
Cost: ${e.parameter["Cost"]}
Estimated Time of Delivery: ${e.parameter["Estimated Time of Delivery"]}
`
    MailApp.sendEmail(to, " Your Order is Shipped! ", message);
    MailApp.sendEmail(toins, " Your Order is Shipped! ", message);
  }

  if(e.parameter["id"] == "OrdersDispatched"){
    var to = "eyrc.vb.1004@gmail.com";  
    var toins = "eyrc.vb.0000@gmail.com";
    var message=`
Hello! Your Order has been dispatched! Contact us incase you have any questions, we are here to help you :)

ORDER SUMMARY:

Order Number: ${e.parameter["Order ID"]}
Item: ${e.parameter["Item"]}
Quantity: ${e.parameter["Dispatch Quantity"]}
Dispatched Date and Time: ${e.parameter["Dispatch Date and Time"]}
City: ${e.parameter["City"]}
Cost: ${e.parameter["Cost"]}
`
    MailApp.sendEmail(to, " Your Order is Dispatched! ", message);
    MailApp.sendEmail(toins, " Your Order is Dispatched! ", message);
  }

  return ContentService.createTextOutput('success');
}

