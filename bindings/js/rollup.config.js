import babel from "@rollup/plugin-babel";
import { nodeResolve } from "@rollup/plugin-node-resolve";
import { terser } from "rollup-plugin-terser";

console.log(`
-------------------------------------
Rollup building bundle for ${process.env.BABEL_ENV}
-------------------------------------
`);

const extensions = [".js", ".ts"];

export default {
  input: "src/index.ts",
  output: [
    ...(process.env.BABEL_ENV === "esmBundled"
      ? [
          {
            file: "lib/bundles/bundle.esm.min.js",
            format: "esm",
            sourcemap: true,
          },
        ]
      : []),
    ...(process.env.BABEL_ENV === "umdBundled"
      ? [
          {
            file: "lib/bundles/bundle.umd.min.js",
            format: "umd",
            name: "my-lib",
            sourcemap: true,
          },
        ]
      : []),
  ],
  plugins: [
    nodeResolve({ extensions }),
    babel({
      babelHelpers: "bundled",
      include: ["src/**/*.ts", "src/**/*.js"],
      extensions,
      exclude: "./node_modules/**",
    }),
    terser(),
  ],
};
