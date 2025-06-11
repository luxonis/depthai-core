const fs = require("fs");

const validate = require("jsonschema").validate;

var args = process.argv.slice(2);
const json = fs.readFileSync(args[0]).toString();
const schema = fs.readFileSync(args[1]).toString();
console.log(validate(JSON.parse(json), JSON.parse(schema)));
