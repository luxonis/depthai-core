const fs = require("fs");

const args = process.argv.slice(2);
const schema = JSON.parse(fs.readFileSync(args[0]).toString());
// not needed anymore, leaving the script here if something else will break
// schema.properties.connections.items.type = "string";

/*
schema.$defs.Head.properties.outputs = undefined;
schema.$defs.Head.required.splice(
  schema.$defs.Head.required.indexOf("outputs"),
  1
);
*/

/*
 * quicktype has problems with this nested union structure
for (let unionType of schema.$defs.Head.properties.outputs.anyOf) {
  if (unionType.additionalProperties !== undefined) {
    unionType.additionalProperties.anyOf[1].items = { type: "string" };
  }
}
*/
fs.writeFileSync(args[1], JSON.stringify(schema, null, 2));
