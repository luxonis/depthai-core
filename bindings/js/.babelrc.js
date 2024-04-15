/**
 * Config fragments to be used by all module
 * format environments
 */
const sharedPresets = ["@babel/preset-typescript"];
const sharedIgnoredFiles = ["src/**/*.test.ts"];
const sharedConfig = {
  ignore: sharedIgnoredFiles,
  presets: sharedPresets,
};
/**
 * Shared configs for bundles (ESM and UMD)
 */
const bundlePresets = [
  [
    "@babel/preset-env",
    {
      targets: "> 0.25%, not dead",
    },
  ],
  ...sharedPresets,
];
const bundleConfig = {
  ...sharedConfig,
  presets: bundlePresets,
};
/**
 * Babel Config
 */
module.exports = {
  env: {
    esmUnbundled: sharedConfig,
    esmBundled: bundleConfig,
    umdBundled: bundleConfig,
    cjs: {
      ignore: sharedIgnoredFiles,
      presets: [
        [
          "@babel/preset-env",
          {
            modules: "commonjs",
          },
        ],
        ...sharedPresets,
      ],
    },
    test: {
      presets: ["@babel/preset-env", ...sharedPresets],
    },
  },
};
