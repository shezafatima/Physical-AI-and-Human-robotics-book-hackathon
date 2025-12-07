// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'chapters/chapter1',
        'chapters/chapter2',
        'chapters/chapter3',
        'chapters/chapter4',
        'chapters/chapter5',
        'chapters/chapter6',
        'chapters/chapter7',
        'chapters/chapter8'
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'advanced/ros2',
        'advanced/simulation',
        'advanced/humanoids'
      ],
    },
  ],
};

export default sidebars;