const fs = require("fs");

const args = process.argv.slice(2);
const schema = JSON.parse(fs.readFileSync(args[0]).toString());
// TODO remove this fix and maybe add others when Connection structure gets defined
schema.properties.connections.items.type = "string";
fs.writeFileSync(args[1], JSON.stringify(schema, null, 2));
