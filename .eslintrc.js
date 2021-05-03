module.exports = {
  plugins: [
    "@typescript-eslint",
    "eslint-comments"
  ],
  extends: [
    "airbnb-typescript/base",
    "plugin:@typescript-eslint/recommended",
    "plugin:@typescript-eslint/recommended-requiring-type-checking",
    "plugin:eslint-comments/recommended",
    "prettier",
  ],
  env: {
    node: true,
    es2021: true
  },
  parserOptions: {
    project: "./tsconfig.json",
  },
  rules: {
    "no-console": "off",
    "no-unused-vars": "off",
    "no-plusplus": "off",
    "no-continue": "off",
    "no-bitwise": "off",
    "import/no-cycle": "off",
    "import/first": "off",
    "class-methods-use-this": "off",
    "lines-between-class-members": "off",
    "@typescript-eslint/no-unused-vars": "off",
    "@typescript-eslint/no-plusplus": "off",
    "@typescript-eslint/no-continue": "off",
    "@typescript-eslint/no-bitwise": "off",
    "@typescript-eslint/lines-between-class-members": "off",
    "@typescript-eslint/class-methods-use-this": "off",
    "@typescript-eslint/no-non-null-assertion": "off",

    // Too restrictive, writing ugly code to defend against a very unlikely scenario: https://eslint.org/docs/rules/no-prototype-builtins
    "no-prototype-builtins": "off",
    // https://basarat.gitbooks.io/typescript/docs/tips/defaultIsBad.html
    "import/prefer-default-export": "off",
    "import/no-default-export": "error",
    // Too restrictive: https://github.com/yannickcr/eslint-plugin-react/blob/master/docs/rules/destructuring-assignment.md
    // "react/destructuring-assignment": "off",
    // Use function hoisting to improve code readability
    "no-use-before-define": "off",
    // "no-use-before-define": [
    //   "error",
    //   { functions: false, classes: true, variables: true },
    // ],
    
    // Allow most functions to rely on type inference. If the function is exported, then `@typescript-eslint/explicit-module-boundary-types` will ensure it's typed.
    "@typescript-eslint/explicit-function-return-type": "off",
    "@typescript-eslint/no-use-before-define": "off",
    // Common abbreviations are known and readable
    "unicorn/prevent-abbreviations": "off",
    // Airbnb prefers forEach
    "unicorn/no-array-for-each": "off",
    // It's not accurate in the monorepo style
    "import/no-extraneous-dependencies": "off",
  },
  overrides: [
    {
      files: ["*.js"],
      rules: {
        // Allow `require()`
        "@typescript-eslint/no-var-requires": "off",
      },
    },
  ],
};