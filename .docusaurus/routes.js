import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-textbook/__docusaurus/debug',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug', '24c'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/config',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/config', 'aa8'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/content',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/content', '622'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/globalData', '666'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/metadata', 'f1b'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/registry', '464'),
    exact: true
  },
  {
    path: '/ai-textbook/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-textbook/__docusaurus/debug/routes', 'd30'),
    exact: true
  },
  {
    path: '/ai-textbook/docs',
    component: ComponentCreator('/ai-textbook/docs', '132'),
    routes: [
      {
        path: '/ai-textbook/docs',
        component: ComponentCreator('/ai-textbook/docs', '9d3'),
        routes: [
          {
            path: '/ai-textbook/docs',
            component: ComponentCreator('/ai-textbook/docs', 'ff0'),
            routes: [
              {
                path: '/ai-textbook/docs/control/week-7-path-planning',
                component: ComponentCreator('/ai-textbook/docs/control/week-7-path-planning', 'da1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/control/week-8-trajectory-planning',
                component: ComponentCreator('/ai-textbook/docs/control/week-8-trajectory-planning', '4d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/control/week-9-mobile-navigation',
                component: ComponentCreator('/ai-textbook/docs/control/week-9-mobile-navigation', '4c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/integration/week-10-learning',
                component: ComponentCreator('/ai-textbook/docs/integration/week-10-learning', '706'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/integration/week-11-deployment',
                component: ComponentCreator('/ai-textbook/docs/integration/week-11-deployment', '213'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/integration/week-12-capstone',
                component: ComponentCreator('/ai-textbook/docs/integration/week-12-capstone', '58b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/intro',
                component: ComponentCreator('/ai-textbook/docs/intro', '41a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/introduction/week-1-embodied-ai',
                component: ComponentCreator('/ai-textbook/docs/introduction/week-1-embodied-ai', '10b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/introduction/week-2-robot-anatomy',
                component: ComponentCreator('/ai-textbook/docs/introduction/week-2-robot-anatomy', '108'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/introduction/week-3-control-systems',
                component: ComponentCreator('/ai-textbook/docs/introduction/week-3-control-systems', '93c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/perception/week-4-computer-vision',
                component: ComponentCreator('/ai-textbook/docs/perception/week-4-computer-vision', '9de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/perception/week-5-3d-perception',
                component: ComponentCreator('/ai-textbook/docs/perception/week-5-3d-perception', '3c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook/docs/perception/week-6-slam',
                component: ComponentCreator('/ai-textbook/docs/perception/week-6-slam', '22c'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-textbook/',
    component: ComponentCreator('/ai-textbook/', 'd09'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
